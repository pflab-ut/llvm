//===- MaxisLongBranch.cpp - Emit long branches ----------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This pass expands a branch or jump instruction into a long branch if its
// offset is too large to fit into its immediate field.
//
// FIXME: Fix pc-region jump instructions which cross 256MB segment boundaries.
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/MaxisABIInfo.h"
#include "MCTargetDesc/MaxisBaseInfo.h"
#include "MCTargetDesc/MaxisMCNaCl.h"
#include "MCTargetDesc/MaxisMCTargetDesc.h"
#include "Maxis.h"
#include "MaxisInstrInfo.h"
#include "MaxisMachineFunction.h"
#include "MaxisSubtarget.h"
#include "MaxisTargetMachine.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/DebugLoc.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Target/TargetMachine.h"
#include <cassert>
#include <cstdint>
#include <iterator>

using namespace llvm;

#define DEBUG_TYPE "maxis-long-branch"

STATISTIC(LongBranches, "Number of long branches.");

static cl::opt<bool> SkipLongBranch(
  "skip-maxis-long-branch",
  cl::init(false),
  cl::desc("MAXIS: Skip long branch pass."),
  cl::Hidden);

static cl::opt<bool> ForceLongBranch(
  "force-maxis-long-branch",
  cl::init(false),
  cl::desc("MAXIS: Expand all branches to long format."),
  cl::Hidden);

namespace {

  using Iter = MachineBasicBlock::iterator;
  using ReverseIter = MachineBasicBlock::reverse_iterator;

  struct MBBInfo {
    uint64_t Size = 0;
    uint64_t Address;
    bool HasLongBranch = false;
    MachineInstr *Br = nullptr;

    MBBInfo() = default;
  };

  class MaxisLongBranch : public MachineFunctionPass {
  public:
    static char ID;

    MaxisLongBranch()
        : MachineFunctionPass(ID), ABI(MaxisABIInfo::Unknown()) {}

    StringRef getPassName() const override { return "Maxis Long Branch"; }

    bool runOnMachineFunction(MachineFunction &F) override;

    MachineFunctionProperties getRequiredProperties() const override {
      return MachineFunctionProperties().set(
          MachineFunctionProperties::Property::NoVRegs);
    }

  private:
    void splitMBB(MachineBasicBlock *MBB);
    void initMBBInfo();
    int64_t computeOffset(const MachineInstr *Br);
    void replaceBranch(MachineBasicBlock &MBB, Iter Br, const DebugLoc &DL,
                       MachineBasicBlock *MBBOpnd);
    void expandToLongBranch(MBBInfo &Info);

    MachineFunction *MF;
    SmallVector<MBBInfo, 16> MBBInfos;
    bool IsPIC;
    MaxisABIInfo ABI;
    unsigned LongBranchSeqSize;
  };

} // end anonymous namespace

char MaxisLongBranch::ID = 0;

/// Iterate over list of Br's operands and search for a MachineBasicBlock
/// operand.
static MachineBasicBlock *getTargetMBB(const MachineInstr &Br) {
  for (unsigned I = 0, E = Br.getDesc().getNumOperands(); I < E; ++I) {
    const MachineOperand &MO = Br.getOperand(I);

    if (MO.isMBB())
      return MO.getMBB();
  }

  llvm_unreachable("This instruction does not have an MBB operand.");
}

// Traverse the list of instructions backwards until a non-debug instruction is
// found or it reaches E.
static ReverseIter getNonDebugInstr(ReverseIter B, const ReverseIter &E) {
  for (; B != E; ++B)
    if (!B->isDebugValue())
      return B;

  return E;
}

// Split MBB if it has two direct jumps/branches.
void MaxisLongBranch::splitMBB(MachineBasicBlock *MBB) {
  ReverseIter End = MBB->rend();
  ReverseIter LastBr = getNonDebugInstr(MBB->rbegin(), End);

  // Return if MBB has no branch instructions.
  if ((LastBr == End) ||
      (!LastBr->isConditionalBranch() && !LastBr->isUnconditionalBranch()))
    return;

  ReverseIter FirstBr = getNonDebugInstr(std::next(LastBr), End);

  // MBB has only one branch instruction if FirstBr is not a branch
  // instruction.
  if ((FirstBr == End) ||
      (!FirstBr->isConditionalBranch() && !FirstBr->isUnconditionalBranch()))
    return;

  assert(!FirstBr->isIndirectBranch() && "Unexpected indirect branch found.");

  // Create a new MBB. Move instructions in MBB to the newly created MBB.
  MachineBasicBlock *NewMBB =
    MF->CreateMachineBasicBlock(MBB->getBasicBlock());

  // Insert NewMBB and fix control flow.
  MachineBasicBlock *Tgt = getTargetMBB(*FirstBr);
  NewMBB->transferSuccessors(MBB);
  NewMBB->removeSuccessor(Tgt, true);
  MBB->addSuccessor(NewMBB);
  MBB->addSuccessor(Tgt);
  MF->insert(std::next(MachineFunction::iterator(MBB)), NewMBB);

  NewMBB->splice(NewMBB->end(), MBB, LastBr.getReverse(), MBB->end());
}

// Fill MBBInfos.
void MaxisLongBranch::initMBBInfo() {
  // Split the MBBs if they have two branches. Each basic block should have at
  // most one branch after this loop is executed.
  for (auto &MBB : *MF)
    splitMBB(&MBB);

  MF->RenumberBlocks();
  MBBInfos.clear();
  MBBInfos.resize(MF->size());

  const MaxisInstrInfo *TII =
      static_cast<const MaxisInstrInfo *>(MF->getSubtarget().getInstrInfo());
  for (unsigned I = 0, E = MBBInfos.size(); I < E; ++I) {
    MachineBasicBlock *MBB = MF->getBlockNumbered(I);

    // Compute size of MBB.
    for (MachineBasicBlock::instr_iterator MI = MBB->instr_begin();
         MI != MBB->instr_end(); ++MI)
      MBBInfos[I].Size += TII->getInstSizeInBytes(*MI);

    // Search for MBB's branch instruction.
    ReverseIter End = MBB->rend();
    ReverseIter Br = getNonDebugInstr(MBB->rbegin(), End);

    if ((Br != End) && !Br->isIndirectBranch() &&
        (Br->isConditionalBranch() || (Br->isUnconditionalBranch() && IsPIC)))
      MBBInfos[I].Br = &*Br;
  }
}

// Compute offset of branch in number of bytes.
int64_t MaxisLongBranch::computeOffset(const MachineInstr *Br) {
  int64_t Offset = 0;
  int ThisMBB = Br->getParent()->getNumber();
  int TargetMBB = getTargetMBB(*Br)->getNumber();

  // Compute offset of a forward branch.
  if (ThisMBB < TargetMBB) {
    for (int N = ThisMBB + 1; N < TargetMBB; ++N)
      Offset += MBBInfos[N].Size;

    return Offset + 4;
  }

  // Compute offset of a backward branch.
  for (int N = ThisMBB; N >= TargetMBB; --N)
    Offset += MBBInfos[N].Size;

  return -Offset + 4;
}

// Replace Br with a branch which has the opposite condition code and a
// MachineBasicBlock operand MBBOpnd.
void MaxisLongBranch::replaceBranch(MachineBasicBlock &MBB, Iter Br,
                                   const DebugLoc &DL,
                                   MachineBasicBlock *MBBOpnd) {
  const MaxisInstrInfo *TII = static_cast<const MaxisInstrInfo *>(
      MBB.getParent()->getSubtarget().getInstrInfo());
  unsigned NewOpc = TII->getOppositeBranchOpc(Br->getOpcode());
  const MCInstrDesc &NewDesc = TII->get(NewOpc);

  MachineInstrBuilder MIB = BuildMI(MBB, Br, DL, NewDesc);

  for (unsigned I = 0, E = Br->getDesc().getNumOperands(); I < E; ++I) {
    MachineOperand &MO = Br->getOperand(I);

    if (!MO.isReg()) {
      assert(MO.isMBB() && "MBB operand expected.");
      break;
    }

    MIB.addReg(MO.getReg());
  }

  MIB.addMBB(MBBOpnd);

  if (Br->hasDelaySlot()) {
    // Bundle the instruction in the delay slot to the newly created branch
    // and erase the original branch.
    assert(Br->isBundledWithSucc());
    MachineBasicBlock::instr_iterator II = Br.getInstrIterator();
    MIBundleBuilder(&*MIB).append((++II)->removeFromBundle());
  }
  Br->eraseFromParent();
}

// Expand branch instructions to long branches.
// TODO: This function has to be fixed for beqz16 and bnez16, because it
// currently assumes that all branches have 16-bit offsets, and will produce
// wrong code if branches whose allowed offsets are [-128, -126, ..., 126]
// are present.
void MaxisLongBranch::expandToLongBranch(MBBInfo &I) {
  MachineBasicBlock::iterator Pos;
  MachineBasicBlock *MBB = I.Br->getParent(), *TgtMBB = getTargetMBB(*I.Br);
  DebugLoc DL = I.Br->getDebugLoc();
  const BasicBlock *BB = MBB->getBasicBlock();
  MachineFunction::iterator FallThroughMBB = ++MachineFunction::iterator(MBB);
  MachineBasicBlock *LongBrMBB = MF->CreateMachineBasicBlock(BB);
  const MaxisSubtarget &Subtarget =
      static_cast<const MaxisSubtarget &>(MF->getSubtarget());
  const MaxisInstrInfo *TII =
      static_cast<const MaxisInstrInfo *>(Subtarget.getInstrInfo());

  MF->insert(FallThroughMBB, LongBrMBB);
  MBB->replaceSuccessor(TgtMBB, LongBrMBB);

  if (IsPIC) {
    MachineBasicBlock *BalTgtMBB = MF->CreateMachineBasicBlock(BB);
    MF->insert(FallThroughMBB, BalTgtMBB);
    LongBrMBB->addSuccessor(BalTgtMBB);
    BalTgtMBB->addSuccessor(TgtMBB);

    // We must select between the MAXIS32r6/MAXIS64r6 BALC (which is a normal
    // instruction) and the pre-MAXIS32r6/MAXIS64r6 definition (which is an
    // pseudo-instruction wrapping BGEZAL).
    const unsigned BalOp =
        Subtarget.hasMaxis32r6()
            ? Subtarget.inMicroMaxisMode() ? Maxis::BALC_MMR6 : Maxis::BALC
            : Maxis::BAL_BR;

    if (!ABI.IsN64()) {
      // Pre R6:
      // $longbr:
      //  addi $sp, $sp, -8
      //  sw $ra, 0($sp)
      //  cati $at, $0, %hi($tgt - $baltgt)
      //  bal $baltgt
      //  addi $at, $at, %lo($tgt - $baltgt)
      // $baltgt:
      //  add $at, $ra, $at
      //  lw $ra, 0($sp)
      //  jr $at
      //  addi $sp, $sp, 8
      // $fallthrough:
      //

      // R6:
      // $longbr:
      //  addi $sp, $sp, -8
      //  sw $ra, 0($sp)
      //  cati $at, $0, %hi($tgt - $baltgt)
      //  addi $at, $at, %lo($tgt - $baltgt)
      //  balc $baltgt
      // $baltgt:
      //  add $at, $ra, $at
      //  lw $ra, 0($sp)
      //  addi $sp, $sp, 8
      //  jic $at, 0
      // $fallthrough:

      Pos = LongBrMBB->begin();

      BuildMI(*LongBrMBB, Pos, DL, TII->get(Maxis::ADDi), Maxis::SP)
        .addReg(Maxis::SP).addImm(-8);
      BuildMI(*LongBrMBB, Pos, DL, TII->get(Maxis::SW)).addReg(Maxis::RA)
        .addReg(Maxis::SP).addImm(0);

      // CATi and ADDi instructions create 32-bit offset of the target basic
      // block from the target of BAL(C) instruction.  We cannot use immediate
      // value for this offset because it cannot be determined accurately when
      // the program has inline assembly statements.  We therefore use the
      // relocation expressions %hi($tgt-$baltgt) and %lo($tgt-$baltgt) which
      // are resolved during the fixup, so the values will always be correct.
      //
      // Since we cannot create %hi($tgt-$baltgt) and %lo($tgt-$baltgt)
      // expressions at this point (it is possible only at the MC layer),
      // we replace CATi and ADDi with pseudo instructions
      // LONG_BRANCH_CATi and LONG_BRANCH_ADDi, and add both basic
      // blocks as operands to these instructions.  When lowering these pseudo
      // instructions to CATi and ADDi in the MC layer, we will create
      // %hi($tgt-$baltgt) and %lo($tgt-$baltgt) expressions and add them as
      // operands to lowered instructions.

      BuildMI(*LongBrMBB, Pos, DL, TII->get(Maxis::LONG_BRANCH_CATi), Maxis::AT).addReg(Maxis::ZERO)
        .addMBB(TgtMBB).addMBB(BalTgtMBB);

      MachineInstrBuilder BalInstr =
          BuildMI(*MF, DL, TII->get(BalOp)).addMBB(BalTgtMBB);
      MachineInstrBuilder ADDiInstr =
          BuildMI(*MF, DL, TII->get(Maxis::LONG_BRANCH_ADDi), Maxis::AT)
              .addReg(Maxis::AT)
              .addMBB(TgtMBB)
              .addMBB(BalTgtMBB);
      if (Subtarget.hasMaxis32r6()) {
        LongBrMBB->insert(Pos, ADDiInstr);
        LongBrMBB->insert(Pos, BalInstr);
      } else {
        LongBrMBB->insert(Pos, BalInstr);
        LongBrMBB->insert(Pos, ADDiInstr);
        LongBrMBB->rbegin()->bundleWithPred();
      }

      Pos = BalTgtMBB->begin();

      BuildMI(*BalTgtMBB, Pos, DL, TII->get(Maxis::ADD), Maxis::AT)
        .addReg(Maxis::RA).addReg(Maxis::AT);
      BuildMI(*BalTgtMBB, Pos, DL, TII->get(Maxis::LW), Maxis::RA)
        .addReg(Maxis::SP).addImm(0);
      if (Subtarget.isTargetNaCl())
        // Bundle-align the target of indirect branch JR.
        TgtMBB->setAlignment(MAXIS_NACL_BUNDLE_ALIGN);

      // In NaCl, modifying the sp is not allowed in branch delay slot.
      // For MAXIS32R6, we can skip using a delay slot branch.
      if (Subtarget.isTargetNaCl() || Subtarget.hasMaxis32r6())
        BuildMI(*BalTgtMBB, Pos, DL, TII->get(Maxis::ADDi), Maxis::SP)
          .addReg(Maxis::SP).addImm(8);

      if (Subtarget.hasMaxis32r6()) {
        const unsigned JICOp =
            Subtarget.inMicroMaxisMode() ? Maxis::JIC_MMR6 : Maxis::JIC;
        BuildMI(*BalTgtMBB, Pos, DL, TII->get(JICOp))
            .addReg(Maxis::AT)
            .addImm(0);

      } else {
        //        BuildMI(*BalTgtMBB, Pos, DL, TII->get(Maxis::JR)).addReg(Maxis::AT);
        BuildMI(*BalTgtMBB, Pos, DL, TII->get(Maxis::JALR)).addReg(Maxis::AT);
        
        if (Subtarget.isTargetNaCl()) {
          BuildMI(*BalTgtMBB, Pos, DL, TII->get(Maxis::NOP));
        } else
          BuildMI(*BalTgtMBB, Pos, DL, TII->get(Maxis::ADDi), Maxis::SP)
              .addReg(Maxis::SP)
              .addImm(8);

        BalTgtMBB->rbegin()->bundleWithPred();
      }
    } else {
      // Pre R6:
      // $longbr:
      //  daddi $sp, $sp, -16
      //  sd $ra, 0($sp)
      //  daddi $at, $zero, %hi($tgt - $baltgt)
      //  dslli $at, $at, 16
      //  bal $baltgt
      //  daddi $at, $at, %lo($tgt - $baltgt)
      // $baltgt:
      //  daddu $at, $ra, $at
      //  ld $ra, 0($sp)
      //  jr64 $at
      //  daddi $sp, $sp, 16
      // $fallthrough:

      // R6:
      // $longbr:
      //  daddi $sp, $sp, -16
      //  sd $ra, 0($sp)
      //  daddi $at, $zero, %hi($tgt - $baltgt)
      //  dslli $at, $at, 16
      //  daddi $at, $at, %lo($tgt - $baltgt)
      //  balc $baltgt
      // $baltgt:
      //  daddu $at, $ra, $at
      //  ld $ra, 0($sp)
      //  daddi $sp, $sp, 16
      //  jic $at, 0
      // $fallthrough:

      // We assume the branch is within-function, and that offset is within
      // +/- 2GB.  High 32 bits will therefore always be zero.

      // Note that this will work even if the offset is negative, because
      // of the +1 modification that's added in that case.  For example, if the
      // offset is -1MB (0xFFFFFFFFFFF00000), the computation for %higher is
      //
      // 0xFFFFFFFFFFF00000 + 0x80008000 = 0x000000007FF08000
      //
      // and the bits [47:32] are zero.  For %highest
      //
      // 0xFFFFFFFFFFF00000 + 0x800080008000 = 0x000080007FF08000
      //
      // and the bits [63:48] are zero.

      Pos = LongBrMBB->begin();

      BuildMI(*LongBrMBB, Pos, DL, TII->get(Maxis::DADDi), Maxis::SP_64)
        .addReg(Maxis::SP_64).addImm(-16);
      BuildMI(*LongBrMBB, Pos, DL, TII->get(Maxis::SD)).addReg(Maxis::RA_64)
        .addReg(Maxis::SP_64).addImm(0);
      BuildMI(*LongBrMBB, Pos, DL, TII->get(Maxis::LONG_BRANCH_DADDi),
              Maxis::AT_64).addReg(Maxis::ZERO_64)
                          .addMBB(TgtMBB, MaxisII::MO_ABS_HI).addMBB(BalTgtMBB);
      BuildMI(*LongBrMBB, Pos, DL, TII->get(Maxis::DSLLi), Maxis::AT_64)
        .addReg(Maxis::AT_64).addImm(16);

      MachineInstrBuilder BalInstr =
          BuildMI(*MF, DL, TII->get(BalOp)).addMBB(BalTgtMBB);
      MachineInstrBuilder DADDiInstr =
          BuildMI(*MF, DL, TII->get(Maxis::LONG_BRANCH_DADDi), Maxis::AT_64)
              .addReg(Maxis::AT_64)
              .addMBB(TgtMBB, MaxisII::MO_ABS_LO)
              .addMBB(BalTgtMBB);
      if (Subtarget.hasMaxis32r6()) {
        LongBrMBB->insert(Pos, DADDiInstr);
        LongBrMBB->insert(Pos, BalInstr);
      } else {
        LongBrMBB->insert(Pos, BalInstr);
        LongBrMBB->insert(Pos, DADDiInstr);
        LongBrMBB->rbegin()->bundleWithPred();
      }

      Pos = BalTgtMBB->begin();

      BuildMI(*BalTgtMBB, Pos, DL, TII->get(Maxis::DADDu), Maxis::AT_64)
        .addReg(Maxis::RA_64).addReg(Maxis::AT_64);
      BuildMI(*BalTgtMBB, Pos, DL, TII->get(Maxis::LD), Maxis::RA_64)
        .addReg(Maxis::SP_64).addImm(0);

      if (Subtarget.hasMaxis64r6()) {
        BuildMI(*BalTgtMBB, Pos, DL, TII->get(Maxis::DADDi), Maxis::SP_64)
            .addReg(Maxis::SP_64)
            .addImm(16);
        BuildMI(*BalTgtMBB, Pos, DL, TII->get(Maxis::JIC64))
            .addReg(Maxis::AT_64)
            .addImm(0);
      } else {
        BuildMI(*BalTgtMBB, Pos, DL, TII->get(Maxis::JR64)).addReg(Maxis::AT_64);
        BuildMI(*BalTgtMBB, Pos, DL, TII->get(Maxis::DADDi), Maxis::SP_64)
            .addReg(Maxis::SP_64)
            .addImm(16);
        BalTgtMBB->rbegin()->bundleWithPred();
      }
    }

    assert(LongBrMBB->size() + BalTgtMBB->size() == LongBranchSeqSize);
  } else {
    // Pre R6:                  R6:
    // $longbr:                 $longbr:
    //  j $tgt                   bc $tgt
    //  nop                     $fallthrough
    // $fallthrough:
    //
    Pos = LongBrMBB->begin();
    LongBrMBB->addSuccessor(TgtMBB);
    if (Subtarget.hasMaxis32r6())
      BuildMI(*LongBrMBB, Pos, DL,
              TII->get(Subtarget.inMicroMaxisMode() ? Maxis::BC_MMR6 : Maxis::BC))
          .addMBB(TgtMBB);
    else
      MIBundleBuilder(*LongBrMBB, Pos)
        .append(BuildMI(*MF, DL, TII->get(Maxis::J)).addMBB(TgtMBB))
        .append(BuildMI(*MF, DL, TII->get(Maxis::NOP)));

    assert(LongBrMBB->size() == LongBranchSeqSize);
  }

  if (I.Br->isUnconditionalBranch()) {
    // Change branch destination.
    assert(I.Br->getDesc().getNumOperands() == 1);
    I.Br->RemoveOperand(0);
    I.Br->addOperand(MachineOperand::CreateMBB(LongBrMBB));
  } else
    // Change branch destination and reverse condition.
    replaceBranch(*MBB, I.Br, DL, &*FallThroughMBB);
}

static void emitGPDisp(MachineFunction &F, const MaxisInstrInfo *TII) {
  MachineBasicBlock &MBB = F.front();
  MachineBasicBlock::iterator I = MBB.begin();
  DebugLoc DL = MBB.findDebugLoc(MBB.begin());
  BuildMI(MBB, I, DL, TII->get(Maxis::CATi), Maxis::V0).addReg(Maxis::ZERO)
    .addExternalSymbol("_gp_disp", MaxisII::MO_ABS_HI);
  BuildMI(MBB, I, DL, TII->get(Maxis::ADDi), Maxis::V0)
    .addReg(Maxis::V0).addExternalSymbol("_gp_disp", MaxisII::MO_ABS_LO);
  MBB.removeLiveIn(Maxis::V0);
}

bool MaxisLongBranch::runOnMachineFunction(MachineFunction &F) {
  const MaxisSubtarget &STI =
      static_cast<const MaxisSubtarget &>(F.getSubtarget());
  const MaxisInstrInfo *TII =
      static_cast<const MaxisInstrInfo *>(STI.getInstrInfo());

  const TargetMachine& TM = F.getTarget();
  IsPIC = TM.isPositionIndependent();
  ABI = static_cast<const MaxisTargetMachine &>(TM).getABI();

  LongBranchSeqSize = IsPIC ? ((ABI.IsN64() || STI.isTargetNaCl()) ? 10 : 9)
                          : (STI.hasMaxis32r6() ? 1 : 2);

  if (STI.inMaxis16Mode() || !STI.enableLongBranchPass())
    return false;
  if (IsPIC && static_cast<const MaxisTargetMachine &>(TM).getABI().IsO32() &&
      F.getInfo<MaxisFunctionInfo>()->globalBaseRegSet())
    emitGPDisp(F, TII);

  if (SkipLongBranch)
    return true;

  MF = &F;
  initMBBInfo();

  SmallVectorImpl<MBBInfo>::iterator I, E = MBBInfos.end();
  bool EverMadeChange = false, MadeChange = true;

  while (MadeChange) {
    MadeChange = false;

    for (I = MBBInfos.begin(); I != E; ++I) {
      // Skip if this MBB doesn't have a branch or the branch has already been
      // converted to a long branch.
      if (!I->Br || I->HasLongBranch)
        continue;

      int ShVal = STI.inMicroMaxisMode() ? 2 : 4;
      int64_t Offset = computeOffset(I->Br) / ShVal;

      if (STI.isTargetNaCl()) {
        // The offset calculation does not include sandboxing instructions
        // that will be added later in the MC layer.  Since at this point we
        // don't know the exact amount of code that "sandboxing" will add, we
        // conservatively estimate that code will not grow more than 100%.
        Offset *= 2;
      }

      // Check if offset fits into 16-bit immediate field of branches.
      if (!ForceLongBranch && isInt<16>(Offset))
        continue;

      I->HasLongBranch = true;
      I->Size += LongBranchSeqSize * 4;
      ++LongBranches;
      EverMadeChange = MadeChange = true;
    }
  }

  if (!EverMadeChange)
    return true;

  // Compute basic block addresses.
  if (IsPIC) {
    uint64_t Address = 0;

    for (I = MBBInfos.begin(); I != E; Address += I->Size, ++I)
      I->Address = Address;
  }

  // Do the expansion.
  for (I = MBBInfos.begin(); I != E; ++I)
    if (I->HasLongBranch)
      expandToLongBranch(*I);

  MF->RenumberBlocks();

  return true;
}

/// createMaxisLongBranchPass - Returns a pass that converts branches to long
/// branches.
FunctionPass *llvm::createMaxisLongBranchPass() { return new MaxisLongBranch(); }
