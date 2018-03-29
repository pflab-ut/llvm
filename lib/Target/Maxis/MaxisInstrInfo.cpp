//===- MaxisInstrInfo.cpp - Maxis Instruction Information -------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the Maxis implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "MaxisInstrInfo.h"
#include "MCTargetDesc/MaxisBaseInfo.h"
#include "MCTargetDesc/MaxisMCTargetDesc.h"
#include "MaxisSubtarget.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/TargetOpcodes.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/DebugLoc.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/Target/TargetMachine.h"
#include <cassert>

using namespace llvm;

#define GET_INSTRINFO_CTOR_DTOR
#include "MaxisGenInstrInfo.inc"

// Pin the vtable to this file.
void MaxisInstrInfo::anchor() {}

MaxisInstrInfo::MaxisInstrInfo(const MaxisSubtarget &STI, unsigned UncondBr)
    : MaxisGenInstrInfo(Maxis::ADJCALLSTACKDOWN, Maxis::ADJCALLSTACKUP),
      Subtarget(STI), UncondBrOpc(UncondBr) {}

const MaxisInstrInfo *MaxisInstrInfo::create(MaxisSubtarget &STI) {
  if (STI.inMaxis16Mode())
    return createMaxis16InstrInfo(STI);

  return createMaxisSEInstrInfo(STI);
}

bool MaxisInstrInfo::isZeroImm(const MachineOperand &op) const {
  return op.isImm() && op.getImm() == 0;
}

/// insertNoop - If data hazard condition is found insert the target nop
/// instruction.
// FIXME: This appears to be dead code.
void MaxisInstrInfo::
insertNoop(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI) const
{
  DebugLoc DL;
  BuildMI(MBB, MI, DL, get(Maxis::NOP));
}

MachineMemOperand *
MaxisInstrInfo::GetMemOperand(MachineBasicBlock &MBB, int FI,
                             MachineMemOperand::Flags Flags) const {
  MachineFunction &MF = *MBB.getParent();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  unsigned Align = MFI.getObjectAlignment(FI);

  return MF.getMachineMemOperand(MachinePointerInfo::getFixedStack(MF, FI),
                                 Flags, MFI.getObjectSize(FI), Align);
}

//===----------------------------------------------------------------------===//
// Branch Analysis
//===----------------------------------------------------------------------===//

void MaxisInstrInfo::AnalyzeCondBr(const MachineInstr *Inst, unsigned Opc,
                                  MachineBasicBlock *&BB,
                                  SmallVectorImpl<MachineOperand> &Cond) const {
  assert(getAnalyzableBrOpc(Opc) && "Not an analyzable branch");
  int NumOp = Inst->getNumExplicitOperands();

  // for both int and fp branches, the last explicit operand is the
  // MBB.
  BB = Inst->getOperand(NumOp-1).getMBB();
  Cond.push_back(MachineOperand::CreateImm(Opc));

  for (int i = 0; i < NumOp-1; i++)
    Cond.push_back(Inst->getOperand(i));
}

bool MaxisInstrInfo::analyzeBranch(MachineBasicBlock &MBB,
                                  MachineBasicBlock *&TBB,
                                  MachineBasicBlock *&FBB,
                                  SmallVectorImpl<MachineOperand> &Cond,
                                  bool AllowModify) const {
  SmallVector<MachineInstr*, 2> BranchInstrs;
  BranchType BT = analyzeBranch(MBB, TBB, FBB, Cond, AllowModify, BranchInstrs);

  return (BT == BT_None) || (BT == BT_Indirect);
}

void MaxisInstrInfo::BuildCondBr(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
                                const DebugLoc &DL,
                                ArrayRef<MachineOperand> Cond) const {
  unsigned Opc = Cond[0].getImm();
  const MCInstrDesc &MCID = get(Opc);
  MachineInstrBuilder MIB = BuildMI(&MBB, DL, MCID);

  for (unsigned i = 1; i < Cond.size(); ++i) {
    assert((Cond[i].isImm() || Cond[i].isReg()) &&
           "Cannot copy operand for conditional branch!");
    MIB.add(Cond[i]);
  }
  MIB.addMBB(TBB);
}

unsigned MaxisInstrInfo::insertBranch(MachineBasicBlock &MBB,
                                     MachineBasicBlock *TBB,
                                     MachineBasicBlock *FBB,
                                     ArrayRef<MachineOperand> Cond,
                                     const DebugLoc &DL,
                                     int *BytesAdded) const {
  // Shouldn't be a fall through.
  assert(TBB && "insertBranch must not be told to insert a fallthrough");
  assert(!BytesAdded && "code size not handled");

  // # of condition operands:
  //  Unconditional branches: 0
  //  Floating point branches: 1 (opc)
  //  Int BranchZero: 2 (opc, reg)
  //  Int Branch: 3 (opc, reg0, reg1)
  assert((Cond.size() <= 3) &&
         "# of Maxis branch conditions must be <= 3!");

  // Two-way Conditional branch.
  if (FBB) {
    BuildCondBr(MBB, TBB, DL, Cond);
    BuildMI(&MBB, DL, get(UncondBrOpc)).addMBB(FBB);
    return 2;
  }

  // One way branch.
  // Unconditional branch.
  if (Cond.empty())
    BuildMI(&MBB, DL, get(UncondBrOpc)).addMBB(TBB);
  else // Conditional branch.
    BuildCondBr(MBB, TBB, DL, Cond);
  return 1;
}

unsigned MaxisInstrInfo::removeBranch(MachineBasicBlock &MBB,
                                     int *BytesRemoved) const {
  assert(!BytesRemoved && "code size not handled");

  MachineBasicBlock::reverse_iterator I = MBB.rbegin(), REnd = MBB.rend();
  unsigned removed = 0;

  // Up to 2 branches are removed.
  // Note that indirect branches are not removed.
  while (I != REnd && removed < 2) {
    // Skip past debug instructions.
    if (I->isDebugValue()) {
      ++I;
      continue;
    }
    if (!getAnalyzableBrOpc(I->getOpcode()))
      break;
    // Remove the branch.
    I->eraseFromParent();
    I = MBB.rbegin();
    ++removed;
  }

  return removed;
}

/// reverseBranchCondition - Return the inverse opcode of the
/// specified Branch instruction.
bool MaxisInstrInfo::reverseBranchCondition(
    SmallVectorImpl<MachineOperand> &Cond) const {
  assert( (Cond.size() && Cond.size() <= 3) &&
          "Invalid Maxis branch condition!");
  Cond[0].setImm(getOppositeBranchOpc(Cond[0].getImm()));
  return false;
}

MaxisInstrInfo::BranchType MaxisInstrInfo::analyzeBranch(
    MachineBasicBlock &MBB, MachineBasicBlock *&TBB, MachineBasicBlock *&FBB,
    SmallVectorImpl<MachineOperand> &Cond, bool AllowModify,
    SmallVectorImpl<MachineInstr *> &BranchInstrs) const {
  MachineBasicBlock::reverse_iterator I = MBB.rbegin(), REnd = MBB.rend();

  // Skip all the debug instructions.
  while (I != REnd && I->isDebugValue())
    ++I;

  if (I == REnd || !isUnpredicatedTerminator(*I)) {
    // This block ends with no branches (it just falls through to its succ).
    // Leave TBB/FBB null.
    TBB = FBB = nullptr;
    return BT_NoBranch;
  }

  MachineInstr *LastInst = &*I;
  unsigned LastOpc = LastInst->getOpcode();
  BranchInstrs.push_back(LastInst);

  // Not an analyzable branch (e.g., indirect jump).
  if (!getAnalyzableBrOpc(LastOpc))
    return LastInst->isIndirectBranch() ? BT_Indirect : BT_None;

  // Get the second to last instruction in the block.
  unsigned SecondLastOpc = 0;
  MachineInstr *SecondLastInst = nullptr;

  // Skip past any debug instruction to see if the second last actual
  // is a branch.
  ++I;
  while (I != REnd && I->isDebugValue())
    ++I;

  if (I != REnd) {
    SecondLastInst = &*I;
    SecondLastOpc = getAnalyzableBrOpc(SecondLastInst->getOpcode());

    // Not an analyzable branch (must be an indirect jump).
    if (isUnpredicatedTerminator(*SecondLastInst) && !SecondLastOpc)
      return BT_None;
  }

  // If there is only one terminator instruction, process it.
  if (!SecondLastOpc) {
    // Unconditional branch.
    if (LastInst->isUnconditionalBranch()) {
      TBB = LastInst->getOperand(0).getMBB();
      return BT_Uncond;
    }

    // Conditional branch
    AnalyzeCondBr(LastInst, LastOpc, TBB, Cond);
    return BT_Cond;
  }

  // If we reached here, there are two branches.
  // If there are three terminators, we don't know what sort of block this is.
  if (++I != REnd && isUnpredicatedTerminator(*I))
    return BT_None;

  BranchInstrs.insert(BranchInstrs.begin(), SecondLastInst);

  // If second to last instruction is an unconditional branch,
  // analyze it and remove the last instruction.
  if (SecondLastInst->isUnconditionalBranch()) {
    // Return if the last instruction cannot be removed.
    if (!AllowModify)
      return BT_None;

    TBB = SecondLastInst->getOperand(0).getMBB();
    LastInst->eraseFromParent();
    BranchInstrs.pop_back();
    return BT_Uncond;
  }

  // Conditional branch followed by an unconditional branch.
  // The last one must be unconditional.
  if (!LastInst->isUnconditionalBranch())
    return BT_None;

  AnalyzeCondBr(SecondLastInst, SecondLastOpc, TBB, Cond);
  FBB = LastInst->getOperand(0).getMBB();

  return BT_CondUncond;
}

/// Return the corresponding compact (no delay slot) form of a branch.
unsigned MaxisInstrInfo::getEquivalentCompactForm(
    const MachineBasicBlock::iterator I) const {
  unsigned Opcode = I->getOpcode();
  bool canUseShortMicroMaxisCTI = false;

  if (Subtarget.inMicroMaxisMode()) {
    switch (Opcode) {
    case Maxis::BNE:
    case Maxis::BNE_MM:
    case Maxis::BEQ:
    case Maxis::BEQ_MM:
    // microMAXIS has NE,EQ branches that do not have delay slots provided one
    // of the operands is zero.
      if (I->getOperand(1).getReg() == Subtarget.getABI().GetZeroReg())
        canUseShortMicroMaxisCTI = true;
      break;
    // For microMAXIS the PseudoReturn and PseudoIndirectBranch are always
    // expanded to JR_MM, so they can be replaced with JRC16_MM.
      //    case Maxis::JR:
    case Maxis::PseudoReturn:
    case Maxis::PseudoIndirectBranch:
    case Maxis::TAILCALLREG:
      canUseShortMicroMaxisCTI = true;
      break;
    }
  }

  // MAXISR6 forbids both operands being the zero register.
  if (Subtarget.hasMaxis32r6() && (I->getNumOperands() > 1) &&
      (I->getOperand(0).isReg() &&
       (I->getOperand(0).getReg() == Maxis::ZERO ||
        I->getOperand(0).getReg() == Maxis::ZERO_64)) &&
      (I->getOperand(1).isReg() &&
       (I->getOperand(1).getReg() == Maxis::ZERO ||
        I->getOperand(1).getReg() == Maxis::ZERO_64)))
    return 0;

  if (Subtarget.hasMaxis32r6() || canUseShortMicroMaxisCTI) {
    switch (Opcode) {
      /*
    case Maxis::B:
      return Maxis::BC;
      */
    case Maxis::BAL:
      return Maxis::BALC;
    case Maxis::BEQ:
    case Maxis::BEQ_MM:
      if (canUseShortMicroMaxisCTI)
        return Maxis::BEQZC_MM;
      else if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return Maxis::BEQC;
    case Maxis::BNE:
    case Maxis::BNE_MM:
      if (canUseShortMicroMaxisCTI)
        return Maxis::BNEZC_MM;
      else if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return Maxis::BNEC;
    case Maxis::BGE:
      if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return Maxis::BGEC;
    case Maxis::BGEU:
      if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return Maxis::BGEUC;
      /*
    case Maxis::BGEZ:
      return Maxis::BGEZC;
      */
      /*
    case Maxis::BGTZ:
      return Maxis::BGTZC;
      */
      /*
    case Maxis::BLEZ:
      return Maxis::BLEZC;
      */
    case Maxis::BLT:
      if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return Maxis::BLTC;
    case Maxis::BLTU:
      if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return Maxis::BLTUC;
      /*
    case Maxis::BLTZ:
      return Maxis::BLTZC;
      */
    case Maxis::BEQ64:
      if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return Maxis::BEQC64;
    case Maxis::BNE64:
      if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return Maxis::BNEC64;
    case Maxis::BGTZ64:
      return Maxis::BGTZC64;
    case Maxis::BGEZ64:
      return Maxis::BGEZC64;
    case Maxis::BLTZ64:
      return Maxis::BLTZC64;
    case Maxis::BLEZ64:
      return Maxis::BLEZC64;
    // For MAXISR6, the instruction 'jic' can be used for these cases. Some
    // tools will accept 'jrc reg' as an alias for 'jic 0, $reg'.
      //    case Maxis::JR:
    case Maxis::PseudoReturn:
    case Maxis::PseudoIndirectBranch:
    case Maxis::TAILCALLREG:
      if (canUseShortMicroMaxisCTI)
        return Maxis::JRC16_MM;
      return Maxis::JIC;
    case Maxis::JALRPseudo:
      return Maxis::JIALC;
    case Maxis::JR64:
    case Maxis::PseudoReturn64:
    case Maxis::PseudoIndirectBranch64:
    case Maxis::TAILCALLREG64:
      return Maxis::JIC64;
    case Maxis::JALR64Pseudo:
      return Maxis::JIALC64;
    default:
      return 0;
    }
  }

  return 0;
}

/// Predicate for distingushing between control transfer instructions and all
/// other instructions for handling forbidden slots. Consider inline assembly
/// as unsafe as well.
bool MaxisInstrInfo::SafeInForbiddenSlot(const MachineInstr &MI) const {
  if (MI.isInlineAsm())
    return false;

  return (MI.getDesc().TSFlags & MaxisII::IsCTI) == 0;
}

/// Predicate for distingushing instructions that have forbidden slots.
bool MaxisInstrInfo::HasForbiddenSlot(const MachineInstr &MI) const {
  return (MI.getDesc().TSFlags & MaxisII::HasForbiddenSlot) != 0;
}

/// Return the number of bytes of code the specified instruction may be.
unsigned MaxisInstrInfo::getInstSizeInBytes(const MachineInstr &MI) const {
  switch (MI.getOpcode()) {
  default:
    return MI.getDesc().getSize();
  case  TargetOpcode::INLINEASM: {       // Inline Asm: Variable size.
    const MachineFunction *MF = MI.getParent()->getParent();
    const char *AsmStr = MI.getOperand(0).getSymbolName();
    return getInlineAsmLength(AsmStr, *MF->getTarget().getMCAsmInfo());
  }
  case Maxis::CONSTPOOL_ENTRY:
    // If this machine instr is a constant pool entry, its size is recorded as
    // operand #2.
    return MI.getOperand(2).getImm();
  }
}

MachineInstrBuilder
MaxisInstrInfo::genInstrWithNewOpc(unsigned NewOpc,
                                  MachineBasicBlock::iterator I) const {
  MachineInstrBuilder MIB;

  // Certain branches have two forms: e.g beq $1, $zero, dest vs beqz $1, dest
  // Pick the zero form of the branch for readable assembly and for greater
  // branch distance in non-microMAXIS mode.
  // Additional MAXISR6 does not permit the use of register $zero for compact
  // branches.
  // FIXME: Certain atomic sequences on maxis64 generate 32bit references to
  // Maxis::ZERO, which is incorrect. This test should be updated to use
  // Subtarget.getABI().GetZeroReg() when those atomic sequences and others
  // are fixed.
  int ZeroOperandPosition = -1;
  bool BranchWithZeroOperand = false;
  if (I->isBranch() && !I->isPseudo()) {
    auto TRI = I->getParent()->getParent()->getSubtarget().getRegisterInfo();
    ZeroOperandPosition = I->findRegisterUseOperandIdx(Maxis::ZERO, false, TRI);
    BranchWithZeroOperand = ZeroOperandPosition != -1;
  }

  if (BranchWithZeroOperand) {
    switch (NewOpc) {
    case Maxis::BEQC:
      NewOpc = Maxis::BEQZC;
      break;
    case Maxis::BNEC:
      NewOpc = Maxis::BNEZC;
      break;
    case Maxis::BGEC:
      NewOpc = Maxis::BGEZC;
      break;
    case Maxis::BLTC:
      NewOpc = Maxis::BLTZC;
      break;
    case Maxis::BEQC64:
      NewOpc = Maxis::BEQZC64;
      break;
    case Maxis::BNEC64:
      NewOpc = Maxis::BNEZC64;
      break;
    }
  }

  MIB = BuildMI(*I->getParent(), I, I->getDebugLoc(), get(NewOpc));

  // For MAXISR6 JI*C requires an immediate 0 as an operand, JIALC(64) an
  // immediate 0 as an operand and requires the removal of it's implicit-def %ra
  // implicit operand as copying the implicit operations of the instructio we're
  // looking at will give us the correct flags.
  if (NewOpc == Maxis::JIC || NewOpc == Maxis::JIALC || NewOpc == Maxis::JIC64 ||
      NewOpc == Maxis::JIALC64) {

    if (NewOpc == Maxis::JIALC || NewOpc == Maxis::JIALC64)
      MIB->RemoveOperand(0);

    for (unsigned J = 0, E = I->getDesc().getNumOperands(); J < E; ++J) {
      MIB.add(I->getOperand(J));
    }

    MIB.addImm(0);

  } else {
    for (unsigned J = 0, E = I->getDesc().getNumOperands(); J < E; ++J) {
      if (BranchWithZeroOperand && (unsigned)ZeroOperandPosition == J)
        continue;

      MIB.add(I->getOperand(J));
    }
  }

  MIB.copyImplicitOps(*I);

  MIB.setMemRefs(I->memoperands_begin(), I->memoperands_end());
  return MIB;
}

bool MaxisInstrInfo::findCommutedOpIndices(MachineInstr &MI, unsigned &SrcOpIdx1,
                                          unsigned &SrcOpIdx2) const {
  assert(!MI.isBundle() &&
         "TargetInstrInfo::findCommutedOpIndices() can't handle bundles");

  const MCInstrDesc &MCID = MI.getDesc();
  if (!MCID.isCommutable())
    return false;

  switch (MI.getOpcode()) {
  case Maxis::DPADD_U_H:
  case Maxis::DPADD_U_W:
  case Maxis::DPADD_U_D:
  case Maxis::DPADD_S_H:
  case Maxis::DPADD_S_W:
  case Maxis::DPADD_S_D:
    // The first operand is both input and output, so it should not commute
    if (!fixCommutedOpIndices(SrcOpIdx1, SrcOpIdx2, 2, 3))
      return false;

    if (!MI.getOperand(SrcOpIdx1).isReg() || !MI.getOperand(SrcOpIdx2).isReg())
      return false;
    return true;
  }
  return TargetInstrInfo::findCommutedOpIndices(MI, SrcOpIdx1, SrcOpIdx2);
}

// ins, ext, dext*, dins have the following constraints:
// X <= pos      <  Y
// X <  size     <= Y
// X <  pos+size <= Y
//
// dinsm and dinsu have the following constraints:
// X <= pos      <  Y
// X <= size     <= Y
// X <  pos+size <= Y
//
// The callee of verifyInsExtInstruction however gives the bounds of
// dins[um] like the other (d)ins (d)ext(um) instructions, so that this
// function doesn't have to vary it's behaviour based on the instruction
// being checked.
static bool verifyInsExtInstruction(const MachineInstr &MI, StringRef &ErrInfo,
                                    const int64_t PosLow, const int64_t PosHigh,
                                    const int64_t SizeLow,
                                    const int64_t SizeHigh,
                                    const int64_t BothLow,
                                    const int64_t BothHigh) {
  MachineOperand MOPos = MI.getOperand(2);
  if (!MOPos.isImm()) {
    ErrInfo = "Position is not an immediate!";
    return false;
  }
  int64_t Pos = MOPos.getImm();
  if (!((PosLow <= Pos) && (Pos < PosHigh))) {
    ErrInfo = "Position operand is out of range!";
    return false;
  }

  MachineOperand MOSize = MI.getOperand(3);
  if (!MOSize.isImm()) {
    ErrInfo = "Size operand is not an immediate!";
    return false;
  }
  int64_t Size = MOSize.getImm();
  if (!((SizeLow < Size) && (Size <= SizeHigh))) {
    ErrInfo = "Size operand is out of range!";
    return false;
  }

  if (!((BothLow < (Pos + Size)) && ((Pos + Size) <= BothHigh))) {
    ErrInfo = "Position + Size is out of range!";
    return false;
  }

  return true;
}

//  Perform target specific instruction verification.
bool MaxisInstrInfo::verifyInstruction(const MachineInstr &MI,
                                      StringRef &ErrInfo) const {
  // Verify that ins and ext instructions are well formed.
  switch (MI.getOpcode()) {
    case Maxis::EXT:
    case Maxis::EXT_MM:
    case Maxis::INS:
    case Maxis::INS_MM:
    case Maxis::DINS:
      return verifyInsExtInstruction(MI, ErrInfo, 0, 32, 0, 32, 0, 32);
    case Maxis::DINSM:
      // The ISA spec has a subtle difference difference between dinsm and dextm
      // in that it says:
      // 2 <= size <= 64 for 'dinsm' but 'dextm' has 32 < size <= 64.
      // To make the bounds checks similar, the range 1 < size <= 64 is checked
      // for 'dinsm'.
      return verifyInsExtInstruction(MI, ErrInfo, 0, 32, 1, 64, 32, 64);
    case Maxis::DINSU:
      // The ISA spec has a subtle difference between dinsu and dextu in that
      // the size range of dinsu is specified as 1 <= size <= 32 whereas size
      // for dextu is 0 < size <= 32. The range checked for dinsu here is
      // 0 < size <= 32, which is equivalent and similar to dextu.
      return verifyInsExtInstruction(MI, ErrInfo, 32, 64, 0, 32, 32, 64);
    case Maxis::DEXT:
      return verifyInsExtInstruction(MI, ErrInfo, 0, 32, 0, 32, 0, 63);
    case Maxis::DEXTM:
      return verifyInsExtInstruction(MI, ErrInfo, 0, 32, 32, 64, 32, 64);
    case Maxis::DEXTU:
      return verifyInsExtInstruction(MI, ErrInfo, 32, 64, 0, 32, 32, 64);
    default:
      return true;
  }

  return true;
}

std::pair<unsigned, unsigned>
MaxisInstrInfo::decomposeMachineOperandsTargetFlags(unsigned TF) const {
  return std::make_pair(TF, 0u);
}

ArrayRef<std::pair<unsigned, const char*>>
MaxisInstrInfo::getSerializableDirectMachineOperandTargetFlags() const {
 using namespace MaxisII;

 static const std::pair<unsigned, const char*> Flags[] = {
    {MO_GOT,          "maxis-got"},
    {MO_GOT_CALL,     "maxis-got-call"},
    {MO_GPREL,        "maxis-gprel"},
    {MO_ABS_HI,       "maxis-abs-hi"},
    {MO_ABS_LO,       "maxis-abs-lo"},
    {MO_TLSGD,        "maxis-tlsgd"},
    {MO_TLSLDM,       "maxis-tlsldm"},
    {MO_DTPREL_HI,    "maxis-dtprel-hi"},
    {MO_DTPREL_LO,    "maxis-dtprel-lo"},
    {MO_GOTTPREL,     "maxis-gottprel"},
    {MO_TPREL_HI,     "maxis-tprel-hi"},
    {MO_TPREL_LO,     "maxis-tprel-lo"},
    {MO_GPOFF_HI,     "maxis-gpoff-hi"},
    {MO_GPOFF_LO,     "maxis-gpoff-lo"},
    {MO_GOT_DISP,     "maxis-got-disp"},
    {MO_GOT_PAGE,     "maxis-got-page"},
    {MO_GOT_OFST,     "maxis-got-ofst"},
    {MO_HIGHER,       "maxis-higher"},
    {MO_HIGHEST,      "maxis-highest"},
    {MO_GOT_HI16,     "maxis-got-hi16"},
    {MO_GOT_LO16,     "maxis-got-lo16"},
    {MO_CALL_HI16,    "maxis-call-hi16"},
    {MO_CALL_LO16,    "maxis-call-lo16"}
  };
  return makeArrayRef(Flags);
}
