//===-- MaxisISelDAGToDAG.cpp - A Dag to Dag Inst Selector for Maxis --------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines an instruction selector for the MAXIS target.
//
//===----------------------------------------------------------------------===//

#include "MaxisISelDAGToDAG.h"
#include "MCTargetDesc/MaxisBaseInfo.h"
#include "Maxis.h"
#include "Maxis16ISelDAGToDAG.h"
#include "MaxisMachineFunction.h"
#include "MaxisRegisterInfo.h"
#include "MaxisSEISelDAGToDAG.h"
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

//===----------------------------------------------------------------------===//
// Instruction Selector Implementation
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// MaxisDAGToDAGISel - MAXIS specific code to select MAXIS machine
// instructions for SelectionDAG operations.
//===----------------------------------------------------------------------===//

bool MaxisDAGToDAGISel::runOnMachineFunction(MachineFunction &MF) {
  Subtarget = &static_cast<const MaxisSubtarget &>(MF.getSubtarget());
  bool Ret = SelectionDAGISel::runOnMachineFunction(MF);

  processFunctionAfterISel(MF);

  return Ret;
}

/// getGlobalBaseReg - Output the instructions required to put the
/// GOT address into a register.
SDNode *MaxisDAGToDAGISel::getGlobalBaseReg() {
  unsigned GlobalBaseReg = MF->getInfo<MaxisFunctionInfo>()->getGlobalBaseReg();
  return CurDAG->getRegister(GlobalBaseReg, getTargetLowering()->getPointerTy(
                                                CurDAG->getDataLayout()))
      .getNode();
}

/// ComplexPattern used on MaxisInstrInfo
/// Used on Maxis Load/Store instructions
bool MaxisDAGToDAGISel::selectAddrRegImm(SDValue Addr, SDValue &Base,
                                        SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool MaxisDAGToDAGISel::selectAddrDefault(SDValue Addr, SDValue &Base,
                                         SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool MaxisDAGToDAGISel::selectIntAddr(SDValue Addr, SDValue &Base,
                                     SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool MaxisDAGToDAGISel::selectIntAddr11MM(SDValue Addr, SDValue &Base,
                                       SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool MaxisDAGToDAGISel::selectIntAddr12MM(SDValue Addr, SDValue &Base,
                                       SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool MaxisDAGToDAGISel::selectIntAddr16MM(SDValue Addr, SDValue &Base,
                                       SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool MaxisDAGToDAGISel::selectIntAddrLSL2MM(SDValue Addr, SDValue &Base,
                                           SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool MaxisDAGToDAGISel::selectIntAddrSImm10(SDValue Addr, SDValue &Base,
                                           SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool MaxisDAGToDAGISel::selectIntAddrSImm10Lsl1(SDValue Addr, SDValue &Base,
                                               SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool MaxisDAGToDAGISel::selectIntAddrSImm10Lsl2(SDValue Addr, SDValue &Base,
                                               SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool MaxisDAGToDAGISel::selectIntAddrSImm10Lsl3(SDValue Addr, SDValue &Base,
                                               SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool MaxisDAGToDAGISel::selectAddr16(SDValue Addr, SDValue &Base,
                                    SDValue &Offset) {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool MaxisDAGToDAGISel::selectAddr16SP(SDValue Addr, SDValue &Base,
                                      SDValue &Offset) {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool MaxisDAGToDAGISel::selectVSplat(SDNode *N, APInt &Imm,
                                    unsigned MinSizeInBits) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool MaxisDAGToDAGISel::selectVSplatUimm1(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool MaxisDAGToDAGISel::selectVSplatUimm2(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool MaxisDAGToDAGISel::selectVSplatUimm3(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool MaxisDAGToDAGISel::selectVSplatUimm4(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool MaxisDAGToDAGISel::selectVSplatUimm5(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool MaxisDAGToDAGISel::selectVSplatUimm6(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool MaxisDAGToDAGISel::selectVSplatUimm8(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool MaxisDAGToDAGISel::selectVSplatSimm5(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool MaxisDAGToDAGISel::selectVSplatUimmPow2(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool MaxisDAGToDAGISel::selectVSplatUimmInvPow2(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool MaxisDAGToDAGISel::selectVSplatMaskL(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool MaxisDAGToDAGISel::selectVSplatMaskR(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

/// Select instructions not customized! Used for
/// expanded, promoted and normal instructions
void MaxisDAGToDAGISel::Select(SDNode *Node) {
  unsigned Opcode = Node->getOpcode();

  // Dump information about the Node being selected
  DEBUG(errs() << "Selecting: "; Node->dump(CurDAG); errs() << "\n");

  // If we have a custom node, we already have selected!
  if (Node->isMachineOpcode()) {
    DEBUG(errs() << "== "; Node->dump(CurDAG); errs() << "\n");
    Node->setNodeId(-1);
    return;
  }

  // See if subclasses can handle this node.
  if (trySelect(Node))
    return;

  switch(Opcode) {
  default: break;

  // Get target GOT address.
  case ISD::GLOBAL_OFFSET_TABLE:
    ReplaceNode(Node, getGlobalBaseReg());
    return;
    /*
#ifndef NDEBUG
  case ISD::LOAD:
  case ISD::STORE:
    assert((Subtarget->systemSupportsUnalignedAccess() ||
            cast<MemSDNode>(Node)->getMemoryVT().getSizeInBits() / 8 <=
            cast<MemSDNode>(Node)->getAlignment()) &&
           "Unexpected unaligned loads/stores.");
    break;
#endif
    */
  }

  // Select the default instruction
  SelectCode(Node);
}

bool MaxisDAGToDAGISel::
SelectInlineAsmMemoryOperand(const SDValue &Op, unsigned ConstraintID,
                             std::vector<SDValue> &OutOps) {
  // All memory constraints can at least accept raw pointers.
  switch(ConstraintID) {
  default:
    llvm_unreachable("Unexpected asm memory constraint");
  case InlineAsm::Constraint_i:
  case InlineAsm::Constraint_m:
  case InlineAsm::Constraint_R:
  case InlineAsm::Constraint_ZC:
    OutOps.push_back(Op);
    return false;
  }
  return true;
}
