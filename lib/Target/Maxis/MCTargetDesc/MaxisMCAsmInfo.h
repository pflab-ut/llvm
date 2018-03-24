//===-- MaxisMCAsmInfo.h - Maxis Asm Info ------------------------*- C++ -*--===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of the MaxisMCAsmInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MAXIS_MCTARGETDESC_MAXISMCASMINFO_H
#define LLVM_LIB_TARGET_MAXIS_MCTARGETDESC_MAXISMCASMINFO_H

#include "llvm/MC/MCAsmInfoELF.h"

namespace llvm {
class Triple;

class MaxisMCAsmInfo : public MCAsmInfoELF {
  void anchor() override;

public:
  explicit MaxisMCAsmInfo(const Triple &TheTriple);
};

} // namespace llvm

#endif
