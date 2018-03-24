//===----------------------------------------------------------------------===//
// Instruction Selector Subtarget Control
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// This file defines a pass used to change the subtarget for the
// Maxis Instruction selector.
//
//===----------------------------------------------------------------------===//

#include "Maxis.h"
#include "MaxisTargetMachine.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "maxis-isel"

namespace {
  class MaxisModuleDAGToDAGISel : public MachineFunctionPass {
  public:
    static char ID;

    MaxisModuleDAGToDAGISel() : MachineFunctionPass(ID) {}

    // Pass Name
    StringRef getPassName() const override {
      return "MAXIS DAG->DAG Pattern Instruction Selection";
    }

    void getAnalysisUsage(AnalysisUsage &AU) const override {
      AU.addRequired<TargetPassConfig>();
      MachineFunctionPass::getAnalysisUsage(AU);
    }

    bool runOnMachineFunction(MachineFunction &MF) override;
  };

  char MaxisModuleDAGToDAGISel::ID = 0;
}

bool MaxisModuleDAGToDAGISel::runOnMachineFunction(MachineFunction &MF) {
  DEBUG(errs() << "In MaxisModuleDAGToDAGISel::runMachineFunction\n");
  auto &TPC = getAnalysis<TargetPassConfig>();
  auto &TM = TPC.getTM<MaxisTargetMachine>();
  TM.resetSubtarget(&MF);
  return false;
}

llvm::FunctionPass *llvm::createMaxisModuleISelDagPass() {
  return new MaxisModuleDAGToDAGISel();
}
