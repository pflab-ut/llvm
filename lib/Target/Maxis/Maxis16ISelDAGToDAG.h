//===---- Maxis16ISelDAGToDAG.h - A Dag to Dag Inst Selector for Maxis ------===//
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

#ifndef LLVM_LIB_TARGET_MAXIS_MAXIS16ISELDAGTODAG_H
#define LLVM_LIB_TARGET_MAXIS_MAXIS16ISELDAGTODAG_H

#include "MaxisISelDAGToDAG.h"

namespace llvm {

class Maxis16DAGToDAGISel : public MaxisDAGToDAGISel {
public:
  explicit Maxis16DAGToDAGISel(MaxisTargetMachine &TM, CodeGenOpt::Level OL)
      : MaxisDAGToDAGISel(TM, OL) {}

private:
  std::pair<SDNode *, SDNode *> selectMULT(SDNode *N, unsigned Opc,
                                           const SDLoc &DL, EVT Ty, bool HasLo,
                                           bool HasHi);

  bool runOnMachineFunction(MachineFunction &MF) override;

  bool selectAddr(bool SPAllowed, SDValue Addr, SDValue &Base,
                  SDValue &Offset);
  bool selectAddr16(SDValue Addr, SDValue &Base,
                    SDValue &Offset) override;
  bool selectAddr16SP(SDValue Addr, SDValue &Base,
                      SDValue &Offset) override;

  bool trySelect(SDNode *Node) override;

  void processFunctionAfterISel(MachineFunction &MF) override;

  // Insert instructions to initialize the global base register in the
  // first MBB of the function.
  void initGlobalBaseReg(MachineFunction &MF);

  void initMaxis16SPAliasReg(MachineFunction &MF);
};

FunctionPass *createMaxis16ISelDag(MaxisTargetMachine &TM,
                                  CodeGenOpt::Level OptLevel);
}

#endif
