//===-- Maxis16ISelDAGToDAG.cpp - A Dag to Dag Inst Selector for Maxis16 ----===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Subclass of MaxisDAGToDAGISel specialized for maxis16.
//
//===----------------------------------------------------------------------===//

#include "Maxis16ISelDAGToDAG.h"
#include "MCTargetDesc/MaxisBaseInfo.h"
#include "Maxis.h"
#include "MaxisMachineFunction.h"
#include "MaxisRegisterInfo.h"
#include "llvm/CodeGen/MachineConstantPool.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAGNodes.h"
#include "llvm/IR/CFG.h"
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

bool Maxis16DAGToDAGISel::runOnMachineFunction(MachineFunction &MF) {
  Subtarget = &static_cast<const MaxisSubtarget &>(MF.getSubtarget());
  if (!Subtarget->inMaxis16Mode())
    return false;
  return MaxisDAGToDAGISel::runOnMachineFunction(MF);
}
/// Select multiply instructions.
std::pair<SDNode *, SDNode *>
Maxis16DAGToDAGISel::selectMULT(SDNode *N, unsigned Opc, const SDLoc &DL, EVT Ty,
                               bool HasLo, bool HasHi) {
  SDNode *Lo = nullptr, *Hi = nullptr;
  SDNode *Mul = CurDAG->getMachineNode(Opc, DL, MVT::Glue, N->getOperand(0),
                                       N->getOperand(1));
  SDValue InFlag = SDValue(Mul, 0);

  if (HasLo) {
    unsigned Opcode = Maxis::Mflo16;
    Lo = CurDAG->getMachineNode(Opcode, DL, Ty, MVT::Glue, InFlag);
    InFlag = SDValue(Lo, 1);
  }
  if (HasHi) {
    unsigned Opcode = Maxis::Mfhi16;
    Hi = CurDAG->getMachineNode(Opcode, DL, Ty, InFlag);
  }
  return std::make_pair(Lo, Hi);
}

void Maxis16DAGToDAGISel::initGlobalBaseReg(MachineFunction &MF) {
  MaxisFunctionInfo *MaxisFI = MF.getInfo<MaxisFunctionInfo>();

  if (!MaxisFI->globalBaseRegSet())
    return;

  MachineBasicBlock &MBB = MF.front();
  MachineBasicBlock::iterator I = MBB.begin();
  MachineRegisterInfo &RegInfo = MF.getRegInfo();
  const TargetInstrInfo &TII = *Subtarget->getInstrInfo();
  DebugLoc DL;
  unsigned V0, V1, V2, GlobalBaseReg = MaxisFI->getGlobalBaseReg();
  const TargetRegisterClass *RC = &Maxis::CPU16RegsRegClass;

  V0 = RegInfo.createVirtualRegister(RC);
  V1 = RegInfo.createVirtualRegister(RC);
  V2 = RegInfo.createVirtualRegister(RC);


  BuildMI(MBB, I, DL, TII.get(Maxis::LiRxImmX16), V0)
      .addExternalSymbol("_gp_disp", MaxisII::MO_ABS_HI);
  BuildMI(MBB, I, DL, TII.get(Maxis::AddiRxPcImmX16), V1)
      .addExternalSymbol("_gp_disp", MaxisII::MO_ABS_LO);

  BuildMI(MBB, I, DL, TII.get(Maxis::SlliX16), V2).addReg(V0).addImm(16);
  BuildMI(MBB, I, DL, TII.get(Maxis::AdduRxRyRz16), GlobalBaseReg)
      .addReg(V1)
      .addReg(V2);
}

void Maxis16DAGToDAGISel::processFunctionAfterISel(MachineFunction &MF) {
  initGlobalBaseReg(MF);
}

bool Maxis16DAGToDAGISel::selectAddr(bool SPAllowed, SDValue Addr, SDValue &Base,
                                    SDValue &Offset) {
  SDLoc DL(Addr);
  EVT ValTy = Addr.getValueType();

  // if Address is FI, get the TargetFrameIndex.
  if (SPAllowed) {
    if (FrameIndexSDNode *FIN = dyn_cast<FrameIndexSDNode>(Addr)) {
      Base = CurDAG->getTargetFrameIndex(FIN->getIndex(), ValTy);
      Offset = CurDAG->getTargetConstant(0, DL, ValTy);
      return true;
    }
  }
  // on PIC code Load GA
  if (Addr.getOpcode() == MaxisISD::Wrapper) {
    Base = Addr.getOperand(0);
    Offset = Addr.getOperand(1);
    return true;
  }
  if (!TM.isPositionIndependent()) {
    if ((Addr.getOpcode() == ISD::TargetExternalSymbol ||
         Addr.getOpcode() == ISD::TargetGlobalAddress))
      return false;
  }
  // Addresses of the form FI+const or FI|const
  if (CurDAG->isBaseWithConstantOffset(Addr)) {
    ConstantSDNode *CN = dyn_cast<ConstantSDNode>(Addr.getOperand(1));
    if (isInt<16>(CN->getSExtValue())) {
      // If the first operand is a FI, get the TargetFI Node
      if (SPAllowed) {
        if (FrameIndexSDNode *FIN =
                dyn_cast<FrameIndexSDNode>(Addr.getOperand(0))) {
          Base = CurDAG->getTargetFrameIndex(FIN->getIndex(), ValTy);
          Offset = CurDAG->getTargetConstant(CN->getZExtValue(), DL, ValTy);
          return true;
        }
      }

      Base = Addr.getOperand(0);
      Offset = CurDAG->getTargetConstant(CN->getZExtValue(), DL, ValTy);
      return true;
    }
  }
  // Operand is a result from an ADD.
  if (Addr.getOpcode() == ISD::ADD) {
    // When loading from constant pools, load the lower address part in
    // the instruction itself. Example, instead of:
    //  lui $2, %hi($CPI1_0)
    //  addi $2, $2, %lo($CPI1_0)
    //  lwc1 $f0, 0($2)
    // Generate:
    //  lui $2, %hi($CPI1_0)
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
  Base = Addr;
  Offset = CurDAG->getTargetConstant(0, DL, ValTy);
  return true;
}

bool Maxis16DAGToDAGISel::selectAddr16(SDValue Addr, SDValue &Base,
                                      SDValue &Offset) {
  return selectAddr(false, Addr, Base, Offset);
}

bool Maxis16DAGToDAGISel::selectAddr16SP(SDValue Addr, SDValue &Base,
                                        SDValue &Offset) {
  return selectAddr(true, Addr, Base, Offset);
}

/// Select instructions not customized! Used for
/// expanded, promoted and normal instructions
bool Maxis16DAGToDAGISel::trySelect(SDNode *Node) {
  unsigned Opcode = Node->getOpcode();
  SDLoc DL(Node);

  ///
  // Instruction Selection not handled by the auto-generated
  // tablegen selection should be handled here.
  ///
  EVT NodeTy = Node->getValueType(0);
  unsigned MultOpc;

  switch (Opcode) {
  default:
    break;

  case ISD::SUBE:
  case ISD::ADDE: {
    SDValue InFlag = Node->getOperand(2), CmpLHS;
    unsigned Opc = InFlag.getOpcode();
    (void)Opc;
    assert(((Opc == ISD::ADDC || Opc == ISD::ADDE) ||
            (Opc == ISD::SUBC || Opc == ISD::SUBE)) &&
           "(ADD|SUB)E flag operand must come from (ADD|SUB)C/E insn");

    unsigned MOp;
    if (Opcode == ISD::ADDE) {
      CmpLHS = InFlag.getValue(0);
      MOp = Maxis::AdduRxRyRz16;
    } else {
      CmpLHS = InFlag.getOperand(0);
      MOp = Maxis::SubuRxRyRz16;
    }

    SDValue Ops[] = {CmpLHS, InFlag.getOperand(1)};

    SDValue LHS = Node->getOperand(0);
    SDValue RHS = Node->getOperand(1);

    EVT VT = LHS.getValueType();

    unsigned Sltu_op = Maxis::SltuRxRyRz16;
    SDNode *Carry = CurDAG->getMachineNode(Sltu_op, DL, VT, Ops);
    unsigned Addu_op = Maxis::AdduRxRyRz16;
    SDNode *AddCarry =
        CurDAG->getMachineNode(Addu_op, DL, VT, SDValue(Carry, 0), RHS);

    CurDAG->SelectNodeTo(Node, MOp, VT, MVT::Glue, LHS, SDValue(AddCarry, 0));
    return true;
  }

  /// Mul with two results
  case ISD::SMUL_LOHI:
  case ISD::UMUL_LOHI: {
    MultOpc = (Opcode == ISD::UMUL_LOHI ? Maxis::MultuRxRy16 : Maxis::MultRxRy16);
    std::pair<SDNode *, SDNode *> LoHi =
        selectMULT(Node, MultOpc, DL, NodeTy, true, true);
    if (!SDValue(Node, 0).use_empty())
      ReplaceUses(SDValue(Node, 0), SDValue(LoHi.first, 0));

    if (!SDValue(Node, 1).use_empty())
      ReplaceUses(SDValue(Node, 1), SDValue(LoHi.second, 0));

    CurDAG->RemoveDeadNode(Node);
    return true;
  }

  case ISD::MULHS:
  case ISD::MULHU: {
    MultOpc = (Opcode == ISD::MULHU ? Maxis::MultuRxRy16 : Maxis::MultRxRy16);
    auto LoHi = selectMULT(Node, MultOpc, DL, NodeTy, false, true);
    ReplaceNode(Node, LoHi.second);
    return true;
  }
  }

  return false;
}

FunctionPass *llvm::createMaxis16ISelDag(MaxisTargetMachine &TM,
                                        CodeGenOpt::Level OptLevel) {
  return new Maxis16DAGToDAGISel(TM, OptLevel);
}
