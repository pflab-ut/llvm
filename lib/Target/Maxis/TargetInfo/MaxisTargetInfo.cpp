//===-- MaxisTargetInfo.cpp - Maxis Target Implementation -------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "Maxis.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/TargetRegistry.h"
using namespace llvm;

Target &llvm::getTheMaxisTarget() {
  static Target TheMaxisTarget;
  return TheMaxisTarget;
}
Target &llvm::getTheMaxiselTarget() {
  static Target TheMaxiselTarget;
  return TheMaxiselTarget;
}
Target &llvm::getTheMaxis64Target() {
  static Target TheMaxis64Target;
  return TheMaxis64Target;
}
Target &llvm::getTheMaxis64elTarget() {
  static Target TheMaxis64elTarget;
  return TheMaxis64elTarget;
}

extern "C" void LLVMInitializeMaxisTargetInfo() {
  RegisterTarget<Triple::maxis,
                 /*HasJIT=*/true>
      X(getTheMaxisTarget(), "maxis", "Maxis", "Maxis");

  RegisterTarget<Triple::maxisel,
                 /*HasJIT=*/true>
      Y(getTheMaxiselTarget(), "maxisel", "Maxisel", "Maxis");

  RegisterTarget<Triple::maxis64,
                 /*HasJIT=*/true>
      A(getTheMaxis64Target(), "maxis64", "Maxis64 [experimental]", "Maxis");

  RegisterTarget<Triple::maxis64el,
                 /*HasJIT=*/true>
      B(getTheMaxis64elTarget(), "maxis64el", "Maxis64el [experimental]", "Maxis");
}
