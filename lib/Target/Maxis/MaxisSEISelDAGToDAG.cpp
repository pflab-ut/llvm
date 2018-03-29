//===-- MaxisSEISelDAGToDAG.cpp - A Dag to Dag Inst Selector for MaxisSE ----===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Subclass of MaxisDAGToDAGISel specialized for maxis32/64.
//
//===----------------------------------------------------------------------===//

#include "MaxisSEISelDAGToDAG.h"
#include "MCTargetDesc/MaxisBaseInfo.h"
#include "Maxis.h"
#include "MaxisAnalyzeImmediate.h"
#include "MaxisMachineFunction.h"
#include "MaxisRegisterInfo.h"
#include "llvm/CodeGen/MachineConstantPool.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAGNodes.h"
#include "llvm/IR/CFG.h"
#include "llvm/IR/Dominators.h"
#include "llvm/IR/GlobalValue.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/Intrinsics.h"
#include "llvm/IR/Type.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetMachine.h"
using namespace llvm;

#define DEBUG_TYPE "maxis-isel"

bool MaxisSEDAGToDAGISel::runOnMachineFunction(MachineFunction &MF) {
  Subtarget = &static_cast<const MaxisSubtarget &>(MF.getSubtarget());
  if (Subtarget->inMaxis16Mode())
    return false;
  return MaxisDAGToDAGISel::runOnMachineFunction(MF);
}

void MaxisSEDAGToDAGISel::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<DominatorTreeWrapperPass>();
  SelectionDAGISel::getAnalysisUsage(AU);
}

void MaxisSEDAGToDAGISel::addDSPCtrlRegOperands(bool IsDef, MachineInstr &MI,
                                               MachineFunction &MF) {
  MachineInstrBuilder MIB(MF, &MI);
  unsigned Mask = MI.getOperand(1).getImm();
  unsigned Flag =
      IsDef ? RegState::ImplicitDefine : RegState::Implicit | RegState::Undef;

  if (Mask & 1)
    MIB.addReg(Maxis::DSPPos, Flag);

  if (Mask & 2)
    MIB.addReg(Maxis::DSPSCount, Flag);

  if (Mask & 4)
    MIB.addReg(Maxis::DSPCarry, Flag);

  if (Mask & 8)
    MIB.addReg(Maxis::DSPOutFlag, Flag);

  if (Mask & 16)
    MIB.addReg(Maxis::DSPCCond, Flag);

  if (Mask & 32)
    MIB.addReg(Maxis::DSPEFI, Flag);
}

unsigned MaxisSEDAGToDAGISel::getMSACtrlReg(const SDValue RegIdx) const {
  switch (cast<ConstantSDNode>(RegIdx)->getZExtValue()) {
  default:
    llvm_unreachable("Could not map int to register");
  case 0: return Maxis::MSAIR;
  case 1: return Maxis::MSACSR;
  case 2: return Maxis::MSAAccess;
  case 3: return Maxis::MSASave;
  case 4: return Maxis::MSAModify;
  case 5: return Maxis::MSARequest;
  case 6: return Maxis::MSAMap;
  case 7: return Maxis::MSAUnmap;
  }
}

bool MaxisSEDAGToDAGISel::replaceUsesWithZeroReg(MachineRegisterInfo *MRI,
                                                const MachineInstr& MI) {
  unsigned DstReg = 0, ZeroReg = 0;

  // Check if MI is "addi $dst, $zero, 0" or "daddi $dst, $zero, 0".
  if ((MI.getOpcode() == Maxis::ADDi) &&
      (MI.getOperand(1).getReg() == Maxis::ZERO) &&
      (MI.getOperand(2).isImm()) &&
      (MI.getOperand(2).getImm() == 0)) {
    DstReg = MI.getOperand(0).getReg();
    ZeroReg = Maxis::ZERO;
  } else if ((MI.getOpcode() == Maxis::DADDi) &&
             (MI.getOperand(1).getReg() == Maxis::ZERO_64) &&
             (MI.getOperand(2).isImm()) &&
             (MI.getOperand(2).getImm() == 0)) {
    DstReg = MI.getOperand(0).getReg();
    ZeroReg = Maxis::ZERO_64;
  }

  if (!DstReg)
    return false;

  // Replace uses with ZeroReg.
  for (MachineRegisterInfo::use_iterator U = MRI->use_begin(DstReg),
       E = MRI->use_end(); U != E;) {
    MachineOperand &MO = *U;
    unsigned OpNo = U.getOperandNo();
    MachineInstr *MI = MO.getParent();
    ++U;

    // Do not replace if it is a phi's operand or is tied to def operand.
    if (MI->isPHI() || MI->isRegTiedToDefOperand(OpNo) || MI->isPseudo())
      continue;

    // Also, we have to check that the register class of the operand
    // contains the zero register.
    if (!MRI->getRegClass(MO.getReg())->contains(ZeroReg))
      continue;

    MO.setReg(ZeroReg);
  }

  return true;
}

void MaxisSEDAGToDAGISel::initGlobalBaseReg(MachineFunction &MF) {
  MaxisFunctionInfo *MaxisFI = MF.getInfo<MaxisFunctionInfo>();

  if (!MaxisFI->globalBaseRegSet())
    return;

  MachineBasicBlock &MBB = MF.front();
  MachineBasicBlock::iterator I = MBB.begin();
  MachineRegisterInfo &RegInfo = MF.getRegInfo();
  const TargetInstrInfo &TII = *Subtarget->getInstrInfo();
  DebugLoc DL;
  unsigned V0, V1, GlobalBaseReg = MaxisFI->getGlobalBaseReg();
  const TargetRegisterClass *RC;
  const MaxisABIInfo &ABI = static_cast<const MaxisTargetMachine &>(TM).getABI();
  RC = (ABI.IsN64()) ? &Maxis::GPR64RegClass : &Maxis::GPR32RegClass;

  V0 = RegInfo.createVirtualRegister(RC);
  V1 = RegInfo.createVirtualRegister(RC);

  if (ABI.IsN64()) {
    MF.getRegInfo().addLiveIn(Maxis::T9_64);
    MBB.addLiveIn(Maxis::T9_64);

    // cati $v0, $0, %hi(%neg(%gp_rel(fname)))
    // daddu $v1, $v0, $t9
    // daddi $globalbasereg, $v1, %lo(%neg(%gp_rel(fname)))
    const GlobalValue *FName = &MF.getFunction();
    BuildMI(MBB, I, DL, TII.get(Maxis::CATi64), V0)
      .addGlobalAddress(FName, 0, MaxisII::MO_GPOFF_HI);
    BuildMI(MBB, I, DL, TII.get(Maxis::DADDu), V1).addReg(V0)
      .addReg(Maxis::T9_64);
    BuildMI(MBB, I, DL, TII.get(Maxis::DADDi), GlobalBaseReg).addReg(V1)
      .addGlobalAddress(FName, 0, MaxisII::MO_GPOFF_LO);
    return;
  }

  if (!MF.getTarget().isPositionIndependent()) {
    // Set global register to __gnu_local_gp.
    //
    // cati   $v0, $0, %hi(__gnu_local_gp)
    // addi $globalbasereg, $v0, %lo(__gnu_local_gp)
    BuildMI(MBB, I, DL, TII.get(Maxis::CATi), V0)
      .addExternalSymbol("__gnu_local_gp", MaxisII::MO_ABS_HI);
    BuildMI(MBB, I, DL, TII.get(Maxis::ADDi), GlobalBaseReg).addReg(V0)
      .addExternalSymbol("__gnu_local_gp", MaxisII::MO_ABS_LO);
    return;
  }

  MF.getRegInfo().addLiveIn(Maxis::T9);
  MBB.addLiveIn(Maxis::T9);

  if (ABI.IsN32()) {
    // cati $v0, $0, %hi(%neg(%gp_rel(fname)))
    // addu $v1, $v0, $t9
    // addi $globalbasereg, $v1, %lo(%neg(%gp_rel(fname)))
    const GlobalValue *FName = &MF.getFunction();
    BuildMI(MBB, I, DL, TII.get(Maxis::CATi), V0)
      .addGlobalAddress(FName, 0, MaxisII::MO_GPOFF_HI);
    BuildMI(MBB, I, DL, TII.get(Maxis::ADDu), V1).addReg(V0).addReg(Maxis::T9);
    BuildMI(MBB, I, DL, TII.get(Maxis::ADDi), GlobalBaseReg).addReg(V1)
      .addGlobalAddress(FName, 0, MaxisII::MO_GPOFF_LO);
    return;
  }

  assert(ABI.IsO32());

  // For O32 ABI, the following instruction sequence is emitted to initialize
  // the global base register:
  //
  //  0. cati  $2, $0, %hi(_gp_disp)
  //  1. addi $2, $2, %lo(_gp_disp)
  //  2. addu  $globalbasereg, $2, $t9
  //
  // We emit only the last instruction here.
  //
  // GNU linker requires that the first two instructions appear at the beginning
  // of a function and no instructions be inserted before or between them.
  // The two instructions are emitted during lowering to MC layer in order to
  // avoid any reordering.
  //
  // Register $2 (Maxis::V0) is added to the list of live-in registers to ensure
  // the value instruction 1 (addi) defines is valid when instruction 2 (addu)
  // reads it.
  MF.getRegInfo().addLiveIn(Maxis::V0);
  MBB.addLiveIn(Maxis::V0);
  BuildMI(MBB, I, DL, TII.get(Maxis::ADDu), GlobalBaseReg)
    .addReg(Maxis::V0).addReg(Maxis::T9);
}

void MaxisSEDAGToDAGISel::processFunctionAfterISel(MachineFunction &MF) {
  initGlobalBaseReg(MF);

  MachineRegisterInfo *MRI = &MF.getRegInfo();

  for (auto &MBB: MF) {
    for (auto &MI: MBB) {
      switch (MI.getOpcode()) {
      case Maxis::RDDSP:
        addDSPCtrlRegOperands(false, MI, MF);
        break;
      case Maxis::WRDSP:
        addDSPCtrlRegOperands(true, MI, MF);
        break;
      default:
        replaceUsesWithZeroReg(MRI, MI);
      }
    }
  }
}

void MaxisSEDAGToDAGISel::selectAddE(SDNode *Node, const SDLoc &DL) const {
  SDValue InFlag = Node->getOperand(2);
  unsigned Opc = InFlag.getOpcode();
  SDValue LHS = Node->getOperand(0), RHS = Node->getOperand(1);
  EVT VT = LHS.getValueType();

  // In the base case, we can rely on the carry bit from the addsc
  // instruction.
  if (Opc == ISD::ADDC) {
    SDValue Ops[3] = {LHS, RHS, InFlag};
    CurDAG->SelectNodeTo(Node, Maxis::ADDWC, VT, MVT::Glue, Ops);
    return;
  }

  assert(Opc == ISD::ADDE && "ISD::ADDE not in a chain of ADDE nodes!");

  // The more complex case is when there is a chain of ISD::ADDE nodes like:
  // (adde (adde (adde (addc a b) c) d) e).
  //
  // The addwc instruction does not write to the carry bit, instead it writes
  // to bit 20 of the dsp control register. To match this series of nodes, each
  // intermediate adde node must be expanded to write the carry bit before the
  // addition.

  // Start by reading the overflow field for addsc and moving the value to the
  // carry field. The usage of 1 here with MaxisISD::RDDSP / Maxis::WRDSP
  // corresponds to reading/writing the entire control register to/from a GPR.

  SDValue CstOne = CurDAG->getTargetConstant(1, DL, MVT::i32);

  SDValue OuFlag = CurDAG->getTargetConstant(20, DL, MVT::i32);

  SDNode *DSPCtrlField =
      CurDAG->getMachineNode(Maxis::RDDSP, DL, MVT::i32, MVT::Glue, CstOne, InFlag);

  SDNode *Carry = CurDAG->getMachineNode(
      Maxis::EXT, DL, MVT::i32, SDValue(DSPCtrlField, 0), OuFlag, CstOne);

  SDValue Ops[4] = {SDValue(DSPCtrlField, 0),
                    CurDAG->getTargetConstant(6, DL, MVT::i32), CstOne,
                    SDValue(Carry, 0)};
  SDNode *DSPCFWithCarry = CurDAG->getMachineNode(Maxis::INS, DL, MVT::i32, Ops);

  // My reading of the the MAXIS DSP 3.01 specification isn't as clear as I
  // would like about whether bit 20 always gets overwritten by addwc.
  // Hence take an extremely conservative view and presume it's sticky. We
  // therefore need to clear it.

  SDValue Zero = CurDAG->getRegister(Maxis::ZERO, MVT::i32);

  SDValue InsOps[4] = {Zero, OuFlag, CstOne, SDValue(DSPCFWithCarry, 0)};
  SDNode *DSPCtrlFinal = CurDAG->getMachineNode(Maxis::INS, DL, MVT::i32, InsOps);

  SDNode *WrDSP = CurDAG->getMachineNode(Maxis::WRDSP, DL, MVT::Glue,
                                         SDValue(DSPCtrlFinal, 0), CstOne);

  SDValue Operands[3] = {LHS, RHS, SDValue(WrDSP, 0)};
  CurDAG->SelectNodeTo(Node, Maxis::ADDWC, VT, MVT::Glue, Operands);
}

/// Match frameindex
bool MaxisSEDAGToDAGISel::selectAddrFrameIndex(SDValue Addr, SDValue &Base,
                                              SDValue &Offset) const {
  if (FrameIndexSDNode *FIN = dyn_cast<FrameIndexSDNode>(Addr)) {
    EVT ValTy = Addr.getValueType();

    Base   = CurDAG->getTargetFrameIndex(FIN->getIndex(), ValTy);
    Offset = CurDAG->getTargetConstant(0, SDLoc(Addr), ValTy);
    return true;
  }
  return false;
}

/// Match frameindex+offset and frameindex|offset
bool MaxisSEDAGToDAGISel::selectAddrFrameIndexOffset(
    SDValue Addr, SDValue &Base, SDValue &Offset, unsigned OffsetBits,
    unsigned ShiftAmount = 0) const {
  if (CurDAG->isBaseWithConstantOffset(Addr)) {
    ConstantSDNode *CN = dyn_cast<ConstantSDNode>(Addr.getOperand(1));
    if (isIntN(OffsetBits + ShiftAmount, CN->getSExtValue())) {
      EVT ValTy = Addr.getValueType();

      // If the first operand is a FI, get the TargetFI Node
      if (FrameIndexSDNode *FIN =
              dyn_cast<FrameIndexSDNode>(Addr.getOperand(0)))
        Base = CurDAG->getTargetFrameIndex(FIN->getIndex(), ValTy);
      else {
        Base = Addr.getOperand(0);
        // If base is a FI, additional offset calculation is done in
        // eliminateFrameIndex, otherwise we need to check the alignment
        if (OffsetToAlignment(CN->getZExtValue(), 1ull << ShiftAmount) != 0)
          return false;
      }

      Offset = CurDAG->getTargetConstant(CN->getZExtValue(), SDLoc(Addr),
                                         ValTy);
      return true;
    }
  }
  return false;
}

/// ComplexPattern used on MaxisInstrInfo
/// Used on Maxis Load/Store instructions
bool MaxisSEDAGToDAGISel::selectAddrRegImm(SDValue Addr, SDValue &Base,
                                          SDValue &Offset) const {
  // if Address is FI, get the TargetFrameIndex.
  if (selectAddrFrameIndex(Addr, Base, Offset))
    return true;

  // on PIC code Load GA
  if (Addr.getOpcode() == MaxisISD::Wrapper) {
    Base   = Addr.getOperand(0);
    Offset = Addr.getOperand(1);
    return true;
  }

  if (!TM.isPositionIndependent()) {
    if ((Addr.getOpcode() == ISD::TargetExternalSymbol ||
        Addr.getOpcode() == ISD::TargetGlobalAddress))
      return false;
  }

  // Addresses of the form FI+const or FI|const
  if (selectAddrFrameIndexOffset(Addr, Base, Offset, 16))
    return true;

  // Operand is a result from an ADD.
  if (Addr.getOpcode() == ISD::ADD) {
    // When loading from constant pools, load the lower address part in
    // the instruction itself. Example, instead of:
    //  cati $2, $0, %hi($CPI1_0)
    //  addi $2, $2, %lo($CPI1_0)
    //  lwc1 $f0, 0($2)
    // Generate:
    //  cati $2, $0, %hi($CPI1_0)
    //  lwc1 $f0, %lo($CPI1_0)($2)
    if (Addr.getOperand(1).getOpcode() == MaxisISD::Lo ||
        Addr.getOperand(1).getOpcode() == MaxisISD::GPRel) {
      SDValue Opnd0 = Addr.getOperand(1).getOperand(0);
      if (isa<ConstantPoolSDNode>(Opnd0) || isa<GlobalAddressSDNode>(Opnd0) ||
          isa<JumpTableSDNode>(Opnd0)) {
        Base = Addr.getOperand(0);
        Offset = Opnd0;
        return true;
      }
    }
  }

  return false;
}

/// ComplexPattern used on MaxisInstrInfo
/// Used on Maxis Load/Store instructions
bool MaxisSEDAGToDAGISel::selectAddrDefault(SDValue Addr, SDValue &Base,
                                           SDValue &Offset) const {
  Base = Addr;
  Offset = CurDAG->getTargetConstant(0, SDLoc(Addr), Addr.getValueType());
  return true;
}

bool MaxisSEDAGToDAGISel::selectIntAddr(SDValue Addr, SDValue &Base,
                                       SDValue &Offset) const {
  return selectAddrRegImm(Addr, Base, Offset) ||
    selectAddrDefault(Addr, Base, Offset);
}

bool MaxisSEDAGToDAGISel::selectAddrRegImm9(SDValue Addr, SDValue &Base,
                                           SDValue &Offset) const {
  if (selectAddrFrameIndex(Addr, Base, Offset))
    return true;

  if (selectAddrFrameIndexOffset(Addr, Base, Offset, 9))
    return true;

  return false;
}

/// Used on microMAXIS LWC2, LDC2, SWC2 and SDC2 instructions (11-bit offset)
bool MaxisSEDAGToDAGISel::selectAddrRegImm11(SDValue Addr, SDValue &Base,
                                            SDValue &Offset) const {
  if (selectAddrFrameIndex(Addr, Base, Offset))
    return true;

  if (selectAddrFrameIndexOffset(Addr, Base, Offset, 11))
    return true;

  return false;
}

/// Used on microMAXIS Load/Store unaligned instructions (12-bit offset)
bool MaxisSEDAGToDAGISel::selectAddrRegImm12(SDValue Addr, SDValue &Base,
                                            SDValue &Offset) const {
  if (selectAddrFrameIndex(Addr, Base, Offset))
    return true;

  if (selectAddrFrameIndexOffset(Addr, Base, Offset, 12))
    return true;

  return false;
}

bool MaxisSEDAGToDAGISel::selectAddrRegImm16(SDValue Addr, SDValue &Base,
                                            SDValue &Offset) const {
  if (selectAddrFrameIndex(Addr, Base, Offset))
    return true;

  if (selectAddrFrameIndexOffset(Addr, Base, Offset, 16))
    return true;

  return false;
}

bool MaxisSEDAGToDAGISel::selectIntAddr11MM(SDValue Addr, SDValue &Base,
                                         SDValue &Offset) const {
  return selectAddrRegImm11(Addr, Base, Offset) ||
    selectAddrDefault(Addr, Base, Offset);
}

bool MaxisSEDAGToDAGISel::selectIntAddr12MM(SDValue Addr, SDValue &Base,
                                         SDValue &Offset) const {
  return selectAddrRegImm12(Addr, Base, Offset) ||
    selectAddrDefault(Addr, Base, Offset);
}

bool MaxisSEDAGToDAGISel::selectIntAddr16MM(SDValue Addr, SDValue &Base,
                                         SDValue &Offset) const {
  return selectAddrRegImm16(Addr, Base, Offset) ||
    selectAddrDefault(Addr, Base, Offset);
}

bool MaxisSEDAGToDAGISel::selectIntAddrLSL2MM(SDValue Addr, SDValue &Base,
                                             SDValue &Offset) const {
  if (selectAddrFrameIndexOffset(Addr, Base, Offset, 7)) {
    if (isa<FrameIndexSDNode>(Base))
      return false;

    if (ConstantSDNode *CN = dyn_cast<ConstantSDNode>(Offset)) {
      unsigned CnstOff = CN->getZExtValue();
      return (CnstOff == (CnstOff & 0x3c));
    }

    return false;
  }

  // For all other cases where "lw" would be selected, don't select "lw16"
  // because it would result in additional instructions to prepare operands.
  if (selectAddrRegImm(Addr, Base, Offset))
    return false;

  return selectAddrDefault(Addr, Base, Offset);
}

bool MaxisSEDAGToDAGISel::selectIntAddrSImm10(SDValue Addr, SDValue &Base,
                                             SDValue &Offset) const {

  if (selectAddrFrameIndex(Addr, Base, Offset))
    return true;

  if (selectAddrFrameIndexOffset(Addr, Base, Offset, 10))
    return true;

  return selectAddrDefault(Addr, Base, Offset);
}

bool MaxisSEDAGToDAGISel::selectIntAddrSImm10Lsl1(SDValue Addr, SDValue &Base,
                                                 SDValue &Offset) const {
  if (selectAddrFrameIndex(Addr, Base, Offset))
    return true;

  if (selectAddrFrameIndexOffset(Addr, Base, Offset, 10, 1))
    return true;

  return selectAddrDefault(Addr, Base, Offset);
}

bool MaxisSEDAGToDAGISel::selectIntAddrSImm10Lsl2(SDValue Addr, SDValue &Base,
                                                 SDValue &Offset) const {
  if (selectAddrFrameIndex(Addr, Base, Offset))
    return true;

  if (selectAddrFrameIndexOffset(Addr, Base, Offset, 10, 2))
    return true;

  return selectAddrDefault(Addr, Base, Offset);
}

bool MaxisSEDAGToDAGISel::selectIntAddrSImm10Lsl3(SDValue Addr, SDValue &Base,
                                                 SDValue &Offset) const {
  if (selectAddrFrameIndex(Addr, Base, Offset))
    return true;

  if (selectAddrFrameIndexOffset(Addr, Base, Offset, 10, 3))
    return true;

  return selectAddrDefault(Addr, Base, Offset);
}

// Select constant vector splats.
//
// Returns true and sets Imm if:
// * MSA is enabled
// * N is a ISD::BUILD_VECTOR representing a constant splat
bool MaxisSEDAGToDAGISel::selectVSplat(SDNode *N, APInt &Imm,
                                      unsigned MinSizeInBits) const {
  if (!Subtarget->hasMSA())
    return false;

  BuildVectorSDNode *Node = dyn_cast<BuildVectorSDNode>(N);

  if (!Node)
    return false;

  APInt SplatValue, SplatUndef;
  unsigned SplatBitSize;
  bool HasAnyUndefs;

  if (!Node->isConstantSplat(SplatValue, SplatUndef, SplatBitSize, HasAnyUndefs,
                             MinSizeInBits, !Subtarget->isLittle()))
    return false;

  Imm = SplatValue;

  return true;
}

// Select constant vector splats.
//
// In addition to the requirements of selectVSplat(), this function returns
// true and sets Imm if:
// * The splat value is the same width as the elements of the vector
// * The splat value fits in an integer with the specified signed-ness and
//   width.
//
// This function looks through ISD::BITCAST nodes.
// TODO: This might not be appropriate for big-endian MSA since BITCAST is
//       sometimes a shuffle in big-endian mode.
//
// It's worth noting that this function is not used as part of the selection
// of ldi.[bhwd] since it does not permit using the wrong-typed ldi.[bhwd]
// instruction to achieve the desired bit pattern. ldi.[bhwd] is selected in
// MaxisSEDAGToDAGISel::selectNode.
bool MaxisSEDAGToDAGISel::
selectVSplatCommon(SDValue N, SDValue &Imm, bool Signed,
                   unsigned ImmBitSize) const {
  APInt ImmValue;
  EVT EltTy = N->getValueType(0).getVectorElementType();

  if (N->getOpcode() == ISD::BITCAST)
    N = N->getOperand(0);

  if (selectVSplat(N.getNode(), ImmValue, EltTy.getSizeInBits()) &&
      ImmValue.getBitWidth() == EltTy.getSizeInBits()) {

    if (( Signed && ImmValue.isSignedIntN(ImmBitSize)) ||
        (!Signed && ImmValue.isIntN(ImmBitSize))) {
      Imm = CurDAG->getTargetConstant(ImmValue, SDLoc(N), EltTy);
      return true;
    }
  }

  return false;
}

// Select constant vector splats.
bool MaxisSEDAGToDAGISel::
selectVSplatUimm1(SDValue N, SDValue &Imm) const {
  return selectVSplatCommon(N, Imm, false, 1);
}

bool MaxisSEDAGToDAGISel::
selectVSplatUimm2(SDValue N, SDValue &Imm) const {
  return selectVSplatCommon(N, Imm, false, 2);
}

bool MaxisSEDAGToDAGISel::
selectVSplatUimm3(SDValue N, SDValue &Imm) const {
  return selectVSplatCommon(N, Imm, false, 3);
}

// Select constant vector splats.
bool MaxisSEDAGToDAGISel::
selectVSplatUimm4(SDValue N, SDValue &Imm) const {
  return selectVSplatCommon(N, Imm, false, 4);
}

// Select constant vector splats.
bool MaxisSEDAGToDAGISel::
selectVSplatUimm5(SDValue N, SDValue &Imm) const {
  return selectVSplatCommon(N, Imm, false, 5);
}

// Select constant vector splats.
bool MaxisSEDAGToDAGISel::
selectVSplatUimm6(SDValue N, SDValue &Imm) const {
  return selectVSplatCommon(N, Imm, false, 6);
}

// Select constant vector splats.
bool MaxisSEDAGToDAGISel::
selectVSplatUimm8(SDValue N, SDValue &Imm) const {
  return selectVSplatCommon(N, Imm, false, 8);
}

// Select constant vector splats.
bool MaxisSEDAGToDAGISel::
selectVSplatSimm5(SDValue N, SDValue &Imm) const {
  return selectVSplatCommon(N, Imm, true, 5);
}

// Select constant vector splats whose value is a power of 2.
//
// In addition to the requirements of selectVSplat(), this function returns
// true and sets Imm if:
// * The splat value is the same width as the elements of the vector
// * The splat value is a power of two.
//
// This function looks through ISD::BITCAST nodes.
// TODO: This might not be appropriate for big-endian MSA since BITCAST is
//       sometimes a shuffle in big-endian mode.
bool MaxisSEDAGToDAGISel::selectVSplatUimmPow2(SDValue N, SDValue &Imm) const {
  APInt ImmValue;
  EVT EltTy = N->getValueType(0).getVectorElementType();

  if (N->getOpcode() == ISD::BITCAST)
    N = N->getOperand(0);

  if (selectVSplat(N.getNode(), ImmValue, EltTy.getSizeInBits()) &&
      ImmValue.getBitWidth() == EltTy.getSizeInBits()) {
    int32_t Log2 = ImmValue.exactLogBase2();

    if (Log2 != -1) {
      Imm = CurDAG->getTargetConstant(Log2, SDLoc(N), EltTy);
      return true;
    }
  }

  return false;
}

// Select constant vector splats whose value only has a consecutive sequence
// of left-most bits set (e.g. 0b11...1100...00).
//
// In addition to the requirements of selectVSplat(), this function returns
// true and sets Imm if:
// * The splat value is the same width as the elements of the vector
// * The splat value is a consecutive sequence of left-most bits.
//
// This function looks through ISD::BITCAST nodes.
// TODO: This might not be appropriate for big-endian MSA since BITCAST is
//       sometimes a shuffle in big-endian mode.
bool MaxisSEDAGToDAGISel::selectVSplatMaskL(SDValue N, SDValue &Imm) const {
  APInt ImmValue;
  EVT EltTy = N->getValueType(0).getVectorElementType();

  if (N->getOpcode() == ISD::BITCAST)
    N = N->getOperand(0);

  if (selectVSplat(N.getNode(), ImmValue, EltTy.getSizeInBits()) &&
      ImmValue.getBitWidth() == EltTy.getSizeInBits()) {
    // Extract the run of set bits starting with bit zero from the bitwise
    // inverse of ImmValue, and test that the inverse of this is the same
    // as the original value.
    if (ImmValue == ~(~ImmValue & ~(~ImmValue + 1))) {

      Imm = CurDAG->getTargetConstant(ImmValue.countPopulation() - 1, SDLoc(N),
                                      EltTy);
      return true;
    }
  }

  return false;
}

// Select constant vector splats whose value only has a consecutive sequence
// of right-most bits set (e.g. 0b00...0011...11).
//
// In addition to the requirements of selectVSplat(), this function returns
// true and sets Imm if:
// * The splat value is the same width as the elements of the vector
// * The splat value is a consecutive sequence of right-most bits.
//
// This function looks through ISD::BITCAST nodes.
// TODO: This might not be appropriate for big-endian MSA since BITCAST is
//       sometimes a shuffle in big-endian mode.
bool MaxisSEDAGToDAGISel::selectVSplatMaskR(SDValue N, SDValue &Imm) const {
  APInt ImmValue;
  EVT EltTy = N->getValueType(0).getVectorElementType();

  if (N->getOpcode() == ISD::BITCAST)
    N = N->getOperand(0);

  if (selectVSplat(N.getNode(), ImmValue, EltTy.getSizeInBits()) &&
      ImmValue.getBitWidth() == EltTy.getSizeInBits()) {
    // Extract the run of set bits starting with bit zero, and test that the
    // result is the same as the original value
    if (ImmValue == (ImmValue & ~(ImmValue + 1))) {
      Imm = CurDAG->getTargetConstant(ImmValue.countPopulation() - 1, SDLoc(N),
                                      EltTy);
      return true;
    }
  }

  return false;
}

bool MaxisSEDAGToDAGISel::selectVSplatUimmInvPow2(SDValue N,
                                                 SDValue &Imm) const {
  APInt ImmValue;
  EVT EltTy = N->getValueType(0).getVectorElementType();

  if (N->getOpcode() == ISD::BITCAST)
    N = N->getOperand(0);

  if (selectVSplat(N.getNode(), ImmValue, EltTy.getSizeInBits()) &&
      ImmValue.getBitWidth() == EltTy.getSizeInBits()) {
    int32_t Log2 = (~ImmValue).exactLogBase2();

    if (Log2 != -1) {
      Imm = CurDAG->getTargetConstant(Log2, SDLoc(N), EltTy);
      return true;
    }
  }

  return false;
}

bool MaxisSEDAGToDAGISel::trySelect(SDNode *Node) {
  unsigned Opcode = Node->getOpcode();
  SDLoc DL(Node);

  ///
  // Instruction Selection not handled by the auto-generated
  // tablegen selection should be handled here.
  ///
  switch(Opcode) {
  default: break;

  case ISD::ADDE: {
    selectAddE(Node, DL);
    return true;
  }

  case ISD::ConstantFP: {
    ConstantFPSDNode *CN = dyn_cast<ConstantFPSDNode>(Node);
    if (Node->getValueType(0) == MVT::f64 && CN->isExactlyValue(+0.0)) {
      if (Subtarget->isGP64bit()) {
        SDValue Zero = CurDAG->getCopyFromReg(CurDAG->getEntryNode(), DL,
                                              Maxis::ZERO_64, MVT::i64);
        ReplaceNode(Node,
                    CurDAG->getMachineNode(Maxis::DMTC1, DL, MVT::f64, Zero));
      } else if (Subtarget->isFP64bit()) {
        SDValue Zero = CurDAG->getCopyFromReg(CurDAG->getEntryNode(), DL,
                                              Maxis::ZERO, MVT::i32);
        ReplaceNode(Node, CurDAG->getMachineNode(Maxis::BuildPairF64_64, DL,
                                                 MVT::f64, Zero, Zero));
      } else {
        SDValue Zero = CurDAG->getCopyFromReg(CurDAG->getEntryNode(), DL,
                                              Maxis::ZERO, MVT::i32);
        ReplaceNode(Node, CurDAG->getMachineNode(Maxis::BuildPairF64, DL,
                                                 MVT::f64, Zero, Zero));
      }
      return true;
    }
    break;
  }

  case ISD::Constant: {
    const ConstantSDNode *CN = dyn_cast<ConstantSDNode>(Node);
    int64_t Imm = CN->getSExtValue();
    unsigned Size = CN->getValueSizeInBits(0);

    if (isInt<32>(Imm))
      break;

    MaxisAnalyzeImmediate AnalyzeImm;

    const MaxisAnalyzeImmediate::InstSeq &Seq =
      AnalyzeImm.Analyze(Imm, Size, false);

    MaxisAnalyzeImmediate::InstSeq::const_iterator Inst = Seq.begin();
    SDLoc DL(CN);
    SDNode *RegOpnd;
    SDValue ImmOpnd = CurDAG->getTargetConstant(SignExtend64<16>(Inst->ImmOpnd),
                                                DL, MVT::i64);

    // The first instruction can be a CATi which is different from other
    // instructions (ADDi, ORI and SLLi) in that it does not have a register
    // operand.
    if (Inst->Opc == Maxis::CATi64)
      RegOpnd = CurDAG->getMachineNode(Inst->Opc, DL, MVT::i64, ImmOpnd);
    else
      RegOpnd =
        CurDAG->getMachineNode(Inst->Opc, DL, MVT::i64,
                               CurDAG->getRegister(Maxis::ZERO_64, MVT::i64),
                               ImmOpnd);

    // The remaining instructions in the sequence are handled here.
    for (++Inst; Inst != Seq.end(); ++Inst) {
      ImmOpnd = CurDAG->getTargetConstant(SignExtend64<16>(Inst->ImmOpnd), DL,
                                          MVT::i64);
      RegOpnd = CurDAG->getMachineNode(Inst->Opc, DL, MVT::i64,
                                       SDValue(RegOpnd, 0), ImmOpnd);
    }

    ReplaceNode(Node, RegOpnd);
    return true;
  }

  case ISD::INTRINSIC_W_CHAIN: {
    switch (cast<ConstantSDNode>(Node->getOperand(1))->getZExtValue()) {
    default:
      break;

    case Intrinsic::maxis_cfcmsa: {
      SDValue ChainIn = Node->getOperand(0);
      SDValue RegIdx = Node->getOperand(2);
      SDValue Reg = CurDAG->getCopyFromReg(ChainIn, DL,
                                           getMSACtrlReg(RegIdx), MVT::i32);
      ReplaceNode(Node, Reg.getNode());
      return true;
    }
    }
    break;
  }

  case ISD::INTRINSIC_WO_CHAIN: {
    switch (cast<ConstantSDNode>(Node->getOperand(0))->getZExtValue()) {
    default:
      break;

    case Intrinsic::maxis_move_v:
      // Like an assignment but will always produce a move.v even if
      // unnecessary.
      ReplaceNode(Node, CurDAG->getMachineNode(Maxis::MOVE_V, DL,
                                               Node->getValueType(0),
                                               Node->getOperand(1)));
      return true;
    }
    break;
  }

  case ISD::INTRINSIC_VOID: {
    switch (cast<ConstantSDNode>(Node->getOperand(1))->getZExtValue()) {
    default:
      break;

    case Intrinsic::maxis_ctcmsa: {
      SDValue ChainIn = Node->getOperand(0);
      SDValue RegIdx  = Node->getOperand(2);
      SDValue Value   = Node->getOperand(3);
      SDValue ChainOut = CurDAG->getCopyToReg(ChainIn, DL,
                                              getMSACtrlReg(RegIdx), Value);
      ReplaceNode(Node, ChainOut.getNode());
      return true;
    }
    }
    break;
  }

  // Manually match MaxisISD::Ins nodes to get the correct instruction. It has
  // to be done in this fashion so that we respect the differences between
  // dins and dinsm, as the difference is that the size operand has the range
  // 0 < size <= 32 for dins while dinsm has the range 2 <= size <= 64 which
  // means SelectionDAGISel would have to test all the operands at once to
  // match the instruction.
  case MaxisISD::Ins: {

    // Sanity checking for the node operands.
    if (Node->getValueType(0) != MVT::i32 && Node->getValueType(0) != MVT::i64)
      return false;

    if (Node->getNumOperands() != 4)
      return false;

    if (Node->getOperand(1)->getOpcode() != ISD::Constant ||
        Node->getOperand(2)->getOpcode() != ISD::Constant)
      return false;

    MVT ResTy = Node->getSimpleValueType(0);
    uint64_t Pos = Node->getConstantOperandVal(1);
    uint64_t Size = Node->getConstantOperandVal(2);

    // Size has to be >0 for 'ins', 'dins' and 'dinsu'.
    if (!Size)
      return false;

    if (Pos + Size > 64)
      return false;

    if (ResTy != MVT::i32 && ResTy != MVT::i64)
      return false;

    unsigned Opcode = 0;
    if (ResTy == MVT::i32) {
      if (Pos + Size <= 32)
        Opcode = Maxis::INS;
    } else {
      if (Pos + Size <= 32)
        Opcode = Maxis::DINS;
      else if (Pos < 32 && 1 < Size)
        Opcode = Maxis::DINSM;
      else
        Opcode = Maxis::DINSU;
    }

    if (Opcode) {
      SDValue Ops[4] = {
          Node->getOperand(0), CurDAG->getTargetConstant(Pos, DL, MVT::i32),
          CurDAG->getTargetConstant(Size, DL, MVT::i32), Node->getOperand(3)};

      ReplaceNode(Node, CurDAG->getMachineNode(Opcode, DL, ResTy, Ops));
      return true;
    }

    return false;
  }

  case MaxisISD::ThreadPointer: {
    EVT PtrVT = getTargetLowering()->getPointerTy(CurDAG->getDataLayout());
    unsigned RdhwrOpc, DestReg;

    if (PtrVT == MVT::i32) {
      RdhwrOpc = Maxis::RDHWR;
      DestReg = Maxis::V1;
    } else {
      RdhwrOpc = Maxis::RDHWR64;
      DestReg = Maxis::V1_64;
    }

    SDNode *Rdhwr =
      CurDAG->getMachineNode(RdhwrOpc, DL,
                             Node->getValueType(0),
                             CurDAG->getRegister(Maxis::HWR29, MVT::i32));
    SDValue Chain = CurDAG->getCopyToReg(CurDAG->getEntryNode(), DL, DestReg,
                                         SDValue(Rdhwr, 0));
    SDValue ResNode = CurDAG->getCopyFromReg(Chain, DL, DestReg, PtrVT);
    ReplaceNode(Node, ResNode.getNode());
    return true;
  }

  case ISD::BUILD_VECTOR: {
    // Select appropriate ldi.[bhwd] instructions for constant splats of
    // 128-bit when MSA is enabled. Fixup any register class mismatches that
    // occur as a result.
    //
    // This allows the compiler to use a wider range of immediates than would
    // otherwise be allowed. If, for example, v4i32 could only use ldi.h then
    // it would not be possible to load { 0x01010101, 0x01010101, 0x01010101,
    // 0x01010101 } without using a constant pool. This would be sub-optimal
    // when // 'ldi.b wd, 1' is capable of producing that bit-pattern in the
    // same set/ of registers. Similarly, ldi.h isn't capable of producing {
    // 0x00000000, 0x00000001, 0x00000000, 0x00000001 } but 'ldi.d wd, 1' can.

    const MaxisABIInfo &ABI =
        static_cast<const MaxisTargetMachine &>(TM).getABI();

    BuildVectorSDNode *BVN = cast<BuildVectorSDNode>(Node);
    APInt SplatValue, SplatUndef;
    unsigned SplatBitSize;
    bool HasAnyUndefs;
    unsigned LdiOp;
    EVT ResVecTy = BVN->getValueType(0);
    EVT ViaVecTy;

    if (!Subtarget->hasMSA() || !BVN->getValueType(0).is128BitVector())
      return false;

    if (!BVN->isConstantSplat(SplatValue, SplatUndef, SplatBitSize,
                              HasAnyUndefs, 8,
                              !Subtarget->isLittle()))
      return false;

    switch (SplatBitSize) {
    default:
      return false;
    case 8:
      LdiOp = Maxis::LDI_B;
      ViaVecTy = MVT::v16i8;
      break;
    case 16:
      LdiOp = Maxis::LDI_H;
      ViaVecTy = MVT::v8i16;
      break;
    case 32:
      LdiOp = Maxis::LDI_W;
      ViaVecTy = MVT::v4i32;
      break;
    case 64:
      LdiOp = Maxis::LDI_D;
      ViaVecTy = MVT::v2i64;
      break;
    }

    SDNode *Res;

    // If we have a signed 10 bit integer, we can splat it directly.
    //
    // If we have something bigger we can synthesize the value into a GPR and
    // splat from there.
    if (SplatValue.isSignedIntN(10)) {
      SDValue Imm = CurDAG->getTargetConstant(SplatValue, DL,
                                              ViaVecTy.getVectorElementType());

      Res = CurDAG->getMachineNode(LdiOp, DL, ViaVecTy, Imm);
    } else if (SplatValue.isSignedIntN(16) &&
               ((ABI.IsO32() && SplatBitSize < 64) ||
                (ABI.IsN32() || ABI.IsN64()))) {
      // Only handle signed 16 bit values when the element size is GPR width.
      // MAXIS64 can handle all the cases but MAXIS32 would need to handle
      // negative cases specifically here. Instead, handle those cases as
      // 64bit values.

      bool Is32BitSplat = ABI.IsO32() || SplatBitSize < 64;
      const unsigned ADDiOp = Is32BitSplat ? Maxis::ADDi : Maxis::DADDi;
      const MVT SplatMVT = Is32BitSplat ? MVT::i32 : MVT::i64;
      SDValue ZeroVal = CurDAG->getRegister(
          Is32BitSplat ? Maxis::ZERO : Maxis::ZERO_64, SplatMVT);

      const unsigned FILLOp =
          SplatBitSize == 16
              ? Maxis::FILL_H
              : (SplatBitSize == 32 ? Maxis::FILL_W
                                    : (SplatBitSize == 64 ? Maxis::FILL_D : 0));

      assert(FILLOp != 0 && "Unknown FILL Op for splat synthesis!");
      assert((!ABI.IsO32() || (FILLOp != Maxis::FILL_D)) &&
             "Attempting to use fill.d on MAXIS32!");

      const unsigned Lo = SplatValue.getLoBits(16).getZExtValue();
      SDValue LoVal = CurDAG->getTargetConstant(Lo, DL, SplatMVT);

      Res = CurDAG->getMachineNode(ADDiOp, DL, SplatMVT, ZeroVal, LoVal);
      Res = CurDAG->getMachineNode(FILLOp, DL, ViaVecTy, SDValue(Res, 0));

    } else if (SplatValue.isSignedIntN(32) && SplatBitSize == 32) {
      // Only handle the cases where the splat size agrees with the size
      // of the SplatValue here.
      const unsigned Lo = SplatValue.getLoBits(16).getZExtValue();
      const unsigned Hi = SplatValue.lshr(16).getLoBits(16).getZExtValue();
      SDValue ZeroVal = CurDAG->getRegister(Maxis::ZERO, MVT::i32);

      SDValue LoVal = CurDAG->getTargetConstant(Lo, DL, MVT::i32);
      SDValue HiVal = CurDAG->getTargetConstant(Hi, DL, MVT::i32);

      if (Hi)
        Res = CurDAG->getMachineNode(Maxis::CATi, DL, MVT::i32, HiVal);

      if (Lo)
        Res = CurDAG->getMachineNode(Maxis::ORi, DL, MVT::i32,
                                     Hi ? SDValue(Res, 0) : ZeroVal, LoVal);

      assert((Hi || Lo) && "Zero case reached 32 bit case splat synthesis!");
      Res = CurDAG->getMachineNode(Maxis::FILL_W, DL, MVT::v4i32, SDValue(Res, 0));

    } else if (SplatValue.isSignedIntN(32) && SplatBitSize == 64 &&
               (ABI.IsN32() || ABI.IsN64())) {
      // N32 and N64 can perform some tricks that O32 can't for signed 32 bit
      // integers due to having 64bit registers. cati will cause the necessary
      // zero/sign extension.
      const unsigned Lo = SplatValue.getLoBits(16).getZExtValue();
      const unsigned Hi = SplatValue.lshr(16).getLoBits(16).getZExtValue();
      SDValue ZeroVal = CurDAG->getRegister(Maxis::ZERO, MVT::i32);

      SDValue LoVal = CurDAG->getTargetConstant(Lo, DL, MVT::i32);
      SDValue HiVal = CurDAG->getTargetConstant(Hi, DL, MVT::i32);

      if (Hi)
        Res = CurDAG->getMachineNode(Maxis::CATi, DL, MVT::i32, HiVal);

      if (Lo)
        Res = CurDAG->getMachineNode(Maxis::ORi, DL, MVT::i32,
                                     Hi ? SDValue(Res, 0) : ZeroVal, LoVal);

      Res = CurDAG->getMachineNode(
              Maxis::SUBREG_TO_REG, DL, MVT::i64,
              CurDAG->getTargetConstant(((Hi >> 15) & 0x1), DL, MVT::i64),
              SDValue(Res, 0),
              CurDAG->getTargetConstant(Maxis::sub_32, DL, MVT::i64));

      Res =
          CurDAG->getMachineNode(Maxis::FILL_D, DL, MVT::v2i64, SDValue(Res, 0));

    } else if (SplatValue.isSignedIntN(64)) {
      // If we have a 64 bit Splat value, we perform a similar sequence to the
      // above:
      //
      // MAXIS32:                            MAXIS64:
      //   cati $res, $0, %highest(val)       cati $res, $0, %highest(val)
      //   ori $res, $res, %higher(val)       ori $res, $res, %higher(val)
      //   cati $res2, $0, %hi(val)           cati $res2, $0, %hi(val)
      //   ori $res2, %res2, %lo(val)         ori $res2, %res2, %lo(val)
      //   $res3 = fill $res2                 dinsu $res, $res2, 0, 32
      //   $res4 = insert.w $res3[1], $res    fill.d $res
      //   splat.d $res4, 0
      //
      // The ability to use dinsu is guaranteed as MSA requires MAXISR5. This saves
      // having to materialize the value by shifts and ors.
      //
      // FIXME: Implement the preferred sequence for MAXIS64R6:
      //
      // MAXIS64R6:
      //   ori $res, $zero, %lo(val)
      //   daui $res, $res, %hi(val)
      //   dahi $res, $res, %higher(val)
      //   dati $res, $res, %highest(cal)
      //   fill.d $res
      //

      const unsigned Lo = SplatValue.getLoBits(16).getZExtValue();
      const unsigned Hi = SplatValue.lshr(16).getLoBits(16).getZExtValue();
      const unsigned Higher = SplatValue.lshr(32).getLoBits(16).getZExtValue();
      const unsigned Highest = SplatValue.lshr(48).getLoBits(16).getZExtValue();

      SDValue LoVal = CurDAG->getTargetConstant(Lo, DL, MVT::i32);
      SDValue HiVal = CurDAG->getTargetConstant(Hi, DL, MVT::i32);
      SDValue HigherVal = CurDAG->getTargetConstant(Higher, DL, MVT::i32);
      SDValue HighestVal = CurDAG->getTargetConstant(Highest, DL, MVT::i32);
      SDValue ZeroVal = CurDAG->getRegister(Maxis::ZERO, MVT::i32);

      // Independent of whether we're targeting MAXIS64 or not, the basic
      // operations are the same. Also, directly use the $zero register if
      // the 16 bit chunk is zero.
      //
      // For optimization purposes we always synthesize the splat value as
      // an i32 value, then if we're targetting MAXIS64, use SUBREG_TO_REG
      // just before combining the values with dinsu to produce an i64. This
      // enables SelectionDAG to aggressively share components of splat values
      // where possible.
      //
      // FIXME: This is the general constant synthesis problem. This code
      //        should be factored out into a class shared between all the
      //        classes that need it. Specifically, for a splat size of 64
      //        bits that's a negative number we can do better than CATi/ORi
      //        for the upper 32bits.

      if (Hi)
        Res = CurDAG->getMachineNode(Maxis::CATi, DL, MVT::i32, HiVal);

      if (Lo)
        Res = CurDAG->getMachineNode(Maxis::ORi, DL, MVT::i32,
                                     Hi ? SDValue(Res, 0) : ZeroVal, LoVal);

      SDNode *HiRes;
      if (Highest)
        HiRes = CurDAG->getMachineNode(Maxis::CATi, DL, MVT::i32, HighestVal);

      if (Higher)
        HiRes = CurDAG->getMachineNode(Maxis::ORi, DL, MVT::i32,
                                       Highest ? SDValue(HiRes, 0) : ZeroVal,
                                       HigherVal);


      if (ABI.IsO32()) {
        Res = CurDAG->getMachineNode(Maxis::FILL_W, DL, MVT::v4i32,
                                     (Hi || Lo) ? SDValue(Res, 0) : ZeroVal);

        Res = CurDAG->getMachineNode(
            Maxis::INSERT_W, DL, MVT::v4i32, SDValue(Res, 0),
            (Highest || Higher) ? SDValue(HiRes, 0) : ZeroVal,
            CurDAG->getTargetConstant(1, DL, MVT::i32));

        const TargetLowering *TLI = getTargetLowering();
        const TargetRegisterClass *RC =
            TLI->getRegClassFor(ViaVecTy.getSimpleVT());

        Res = CurDAG->getMachineNode(
            Maxis::COPY_TO_REGCLASS, DL, ViaVecTy, SDValue(Res, 0),
            CurDAG->getTargetConstant(RC->getID(), DL, MVT::i32));

        Res = CurDAG->getMachineNode(
            Maxis::SPLATI_D, DL, MVT::v2i64, SDValue(Res, 0),
            CurDAG->getTargetConstant(0, DL, MVT::i32));
      } else if (ABI.IsN64() || ABI.IsN32()) {

        SDValue Zero64Val = CurDAG->getRegister(Maxis::ZERO_64, MVT::i64);
        const bool HiResNonZero = Highest || Higher;
        const bool ResNonZero = Hi || Lo;

        if (HiResNonZero)
          HiRes = CurDAG->getMachineNode(
              Maxis::SUBREG_TO_REG, DL, MVT::i64,
              CurDAG->getTargetConstant(((Highest >> 15) & 0x1), DL, MVT::i64),
              SDValue(HiRes, 0),
              CurDAG->getTargetConstant(Maxis::sub_32, DL, MVT::i64));

        if (ResNonZero)
          Res = CurDAG->getMachineNode(
              Maxis::SUBREG_TO_REG, DL, MVT::i64,
              CurDAG->getTargetConstant(((Hi >> 15) & 0x1), DL, MVT::i64),
              SDValue(Res, 0),
              CurDAG->getTargetConstant(Maxis::sub_32, DL, MVT::i64));

        // We have 3 cases:
        //   The HiRes is nonzero but Res is $zero  => dslli32 HiRes, 0
        //   The Res is nonzero but HiRes is $zero  => dinsu Res, $zero, 32, 32
        //   Both are non zero                      => dinsu Res, HiRes, 32, 32
        //
        // The obvious "missing" case is when both are zero, but that case is
        // handled by the ldi case.
        if (ResNonZero) {
          IntegerType *Int32Ty =
              IntegerType::get(MF->getFunction().getContext(), 32);
          const ConstantInt *Const32 = ConstantInt::get(Int32Ty, 32);
          SDValue Ops[4] = {HiResNonZero ? SDValue(HiRes, 0) : Zero64Val,
                            CurDAG->getConstant(*Const32, DL, MVT::i32),
                            CurDAG->getConstant(*Const32, DL, MVT::i32),
                            SDValue(Res, 0)};

          Res = CurDAG->getMachineNode(Maxis::DINSU, DL, MVT::i64, Ops);
        } else if (HiResNonZero) {
          Res = CurDAG->getMachineNode(
              Maxis::DSLLi32, DL, MVT::i64, SDValue(HiRes, 0),
              CurDAG->getTargetConstant(0, DL, MVT::i32));
        } else
          llvm_unreachable(
              "Zero splat value handled by non-zero 64bit splat synthesis!");

        Res = CurDAG->getMachineNode(Maxis::FILL_D, DL, MVT::v2i64, SDValue(Res, 0));
      } else
        llvm_unreachable("Unknown ABI in MaxisISelDAGToDAG!");

    } else
      return false;

    if (ResVecTy != ViaVecTy) {
      // If LdiOp is writing to a different register class to ResVecTy, then
      // fix it up here. This COPY_TO_REGCLASS should never cause a move.v
      // since the source and destination register sets contain the same
      // registers.
      const TargetLowering *TLI = getTargetLowering();
      MVT ResVecTySimple = ResVecTy.getSimpleVT();
      const TargetRegisterClass *RC = TLI->getRegClassFor(ResVecTySimple);
      Res = CurDAG->getMachineNode(Maxis::COPY_TO_REGCLASS, DL,
                                   ResVecTy, SDValue(Res, 0),
                                   CurDAG->getTargetConstant(RC->getID(), DL,
                                                             MVT::i32));
    }

    ReplaceNode(Node, Res);
    return true;
  }

  }

  return false;
}

bool MaxisSEDAGToDAGISel::
SelectInlineAsmMemoryOperand(const SDValue &Op, unsigned ConstraintID,
                             std::vector<SDValue> &OutOps) {
  SDValue Base, Offset;

  switch(ConstraintID) {
  default:
    llvm_unreachable("Unexpected asm memory constraint");
  // All memory constraints can at least accept raw pointers.
  case InlineAsm::Constraint_i:
    OutOps.push_back(Op);
    OutOps.push_back(CurDAG->getTargetConstant(0, SDLoc(Op), MVT::i32));
    return false;
  case InlineAsm::Constraint_m:
    if (selectAddrRegImm16(Op, Base, Offset)) {
      OutOps.push_back(Base);
      OutOps.push_back(Offset);
      return false;
    }
    OutOps.push_back(Op);
    OutOps.push_back(CurDAG->getTargetConstant(0, SDLoc(Op), MVT::i32));
    return false;
  case InlineAsm::Constraint_R:
    // The 'R' constraint is supposed to be much more complicated than this.
    // However, it's becoming less useful due to architectural changes and
    // ought to be replaced by other constraints such as 'ZC'.
    // For now, support 9-bit signed offsets which is supportable by all
    // subtargets for all instructions.
    if (selectAddrRegImm9(Op, Base, Offset)) {
      OutOps.push_back(Base);
      OutOps.push_back(Offset);
      return false;
    }
    OutOps.push_back(Op);
    OutOps.push_back(CurDAG->getTargetConstant(0, SDLoc(Op), MVT::i32));
    return false;
  case InlineAsm::Constraint_ZC:
    // ZC matches whatever the pref, ll, and sc instructions can handle for the
    // given subtarget.
    if (Subtarget->inMicroMaxisMode()) {
      // On microMAXIS, they can handle 12-bit offsets.
      if (selectAddrRegImm12(Op, Base, Offset)) {
        OutOps.push_back(Base);
        OutOps.push_back(Offset);
        return false;
      }
    } else if (Subtarget->hasMaxis32r6()) {
      // On MAXIS32r6/MAXIS64r6, they can only handle 9-bit offsets.
      if (selectAddrRegImm9(Op, Base, Offset)) {
        OutOps.push_back(Base);
        OutOps.push_back(Offset);
        return false;
      }
    } else if (selectAddrRegImm16(Op, Base, Offset)) {
      // Prior to MAXIS32r6/MAXIS64r6, they can handle 16-bit offsets.
      OutOps.push_back(Base);
      OutOps.push_back(Offset);
      return false;
    }
    // In all cases, 0-bit offsets are acceptable.
    OutOps.push_back(Op);
    OutOps.push_back(CurDAG->getTargetConstant(0, SDLoc(Op), MVT::i32));
    return false;
  }
  return true;
}

FunctionPass *llvm::createMaxisSEISelDag(MaxisTargetMachine &TM,
                                        CodeGenOpt::Level OptLevel) {
  return new MaxisSEDAGToDAGISel(TM, OptLevel);
}
