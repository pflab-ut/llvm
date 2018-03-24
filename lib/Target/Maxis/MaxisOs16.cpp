//===---- MaxisOs16.cpp for Maxis Option -Os16                       --------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines an optimization phase for the MAXIS target.
//
//===----------------------------------------------------------------------===//

#include "Maxis.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "maxis-os16"

static cl::opt<std::string> Maxis32FunctionMask(
  "maxis32-function-mask",
  cl::init(""),
  cl::desc("Force function to be maxis32"),
  cl::Hidden);

namespace {
  class MaxisOs16 : public ModulePass {
  public:
    static char ID;

    MaxisOs16() : ModulePass(ID) {}

    StringRef getPassName() const override { return "MAXIS Os16 Optimization"; }

    bool runOnModule(Module &M) override;
  };

  char MaxisOs16::ID = 0;
}

// Figure out if we need float point based on the function signature.
// We need to move variables in and/or out of floating point
// registers because of the ABI
//
static  bool needsFPFromSig(Function &F) {
  Type* RetType = F.getReturnType();
  switch (RetType->getTypeID()) {
  case Type::FloatTyID:
  case Type::DoubleTyID:
    return true;
  default:
    ;
  }
  if (F.arg_size() >=1) {
    Argument &Arg = *F.arg_begin();
    switch (Arg.getType()->getTypeID()) {
    case Type::FloatTyID:
    case Type::DoubleTyID:
      return true;
    default:
      ;
    }
  }
  return false;
}

// Figure out if the function will need floating point operations
//
static bool needsFP(Function &F) {
  if (needsFPFromSig(F))
    return true;
  for (Function::const_iterator BB = F.begin(), E = F.end(); BB != E; ++BB)
    for (BasicBlock::const_iterator I = BB->begin(), E = BB->end();
         I != E; ++I) {
      const Instruction &Inst = *I;
      switch (Inst.getOpcode()) {
      case Instruction::FAdd:
      case Instruction::FSub:
      case Instruction::FMul:
      case Instruction::FDiv:
      case Instruction::FRem:
      case Instruction::FPToUI:
      case Instruction::FPToSI:
      case Instruction::UIToFP:
      case Instruction::SIToFP:
      case Instruction::FPTrunc:
      case Instruction::FPExt:
      case Instruction::FCmp:
        return true;
      default:
        ;
      }
      if (const CallInst *CI = dyn_cast<CallInst>(I)) {
        DEBUG(dbgs() << "Working on call" << "\n");
        Function &F_ =  *CI->getCalledFunction();
        if (needsFPFromSig(F_))
          return true;
      }
    }
  return false;
}


bool MaxisOs16::runOnModule(Module &M) {
  bool usingMask = Maxis32FunctionMask.length() > 0;
  bool doneUsingMask = false; // this will make it stop repeating

  DEBUG(dbgs() << "Run on Module MaxisOs16 \n" << Maxis32FunctionMask << "\n");
  if (usingMask)
    DEBUG(dbgs() << "using mask \n" << Maxis32FunctionMask << "\n");

  unsigned int functionIndex = 0;
  bool modified = false;

  for (auto &F : M) {
    if (F.isDeclaration())
      continue;

    DEBUG(dbgs() << "Working on " << F.getName() << "\n");
    if (usingMask) {
      if (!doneUsingMask) {
        if (functionIndex == Maxis32FunctionMask.length())
          functionIndex = 0;
        switch (Maxis32FunctionMask[functionIndex]) {
        case '1':
          DEBUG(dbgs() << "mask forced maxis32: " << F.getName() << "\n");
          F.addFnAttr("nomaxis16");
          break;
        case '.':
          doneUsingMask = true;
          break;
        default:
          break;
        }
        functionIndex++;
      }
    }
    else {
      if (needsFP(F)) {
        DEBUG(dbgs() << "os16 forced maxis32: " << F.getName() << "\n");
        F.addFnAttr("nomaxis16");
      }
      else {
        DEBUG(dbgs() << "os16 forced maxis16: " << F.getName() << "\n");
        F.addFnAttr("maxis16");
      }
    }
  }

  return modified;
}

ModulePass *llvm::createMaxisOs16Pass() { return new MaxisOs16(); }
