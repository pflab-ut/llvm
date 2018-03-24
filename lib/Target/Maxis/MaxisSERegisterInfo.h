//===-- MaxisSERegisterInfo.h - Maxis32/64 Register Information ---*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the Maxis32/64 implementation of the TargetRegisterInfo
// class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MAXIS_MAXISSEREGISTERINFO_H
#define LLVM_LIB_TARGET_MAXIS_MAXISSEREGISTERINFO_H

#include "MaxisRegisterInfo.h"

namespace llvm {
class MaxisSEInstrInfo;

class MaxisSERegisterInfo : public MaxisRegisterInfo {
public:
  MaxisSERegisterInfo();

  bool requiresRegisterScavenging(const MachineFunction &MF) const override;

  bool requiresFrameIndexScavenging(const MachineFunction &MF) const override;

  const TargetRegisterClass *intRegClass(unsigned Size) const override;

private:
  void eliminateFI(MachineBasicBlock::iterator II, unsigned OpNo,
                   int FrameIndex, uint64_t StackSize,
                   int64_t SPOffset) const override;
};

} // end namespace llvm

#endif
