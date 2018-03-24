//===-- Maxis.h - Top-level interface for Maxis representation ----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in
// the LLVM Maxis back-end.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MAXIS_MAXIS_H
#define LLVM_LIB_TARGET_MAXIS_MAXIS_H

#include "MCTargetDesc/MaxisMCTargetDesc.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {
  class MaxisTargetMachine;
  class ModulePass;
  class FunctionPass;

  ModulePass *createMaxisOs16Pass();
  ModulePass *createMaxis16HardFloatPass();

  FunctionPass *createMaxisModuleISelDagPass();
  FunctionPass *createMaxisOptimizePICCallPass();
  FunctionPass *createMaxisDelaySlotFillerPass();
  FunctionPass *createMaxisHazardSchedule();
  FunctionPass *createMaxisLongBranchPass();
  FunctionPass *createMaxisConstantIslandPass();
  FunctionPass *createMicroMaxisSizeReductionPass();
} // end namespace llvm;

#endif
