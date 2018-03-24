//===-- MaxisFrameLowering.h - Define frame lowering for Maxis ----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
//
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MAXIS_MAXISFRAMELOWERING_H
#define LLVM_LIB_TARGET_MAXIS_MAXISFRAMELOWERING_H

#include "Maxis.h"
#include "llvm/CodeGen/TargetFrameLowering.h"

namespace llvm {
  class MaxisSubtarget;

class MaxisFrameLowering : public TargetFrameLowering {
protected:
  const MaxisSubtarget &STI;

public:
  explicit MaxisFrameLowering(const MaxisSubtarget &sti, unsigned Alignment)
    : TargetFrameLowering(StackGrowsDown, Alignment, 0, Alignment), STI(sti) {}

  static const MaxisFrameLowering *create(const MaxisSubtarget &ST);

  bool hasFP(const MachineFunction &MF) const override;

  bool hasBP(const MachineFunction &MF) const;

  bool isFPCloseToIncomingSP() const override { return false; }

  MachineBasicBlock::iterator
  eliminateCallFramePseudoInstr(MachineFunction &MF,
                                MachineBasicBlock &MBB,
                                MachineBasicBlock::iterator I) const override;

protected:
  uint64_t estimateStackSize(const MachineFunction &MF) const;
};

/// Create MaxisFrameLowering objects.
const MaxisFrameLowering *createMaxis16FrameLowering(const MaxisSubtarget &ST);
const MaxisFrameLowering *createMaxisSEFrameLowering(const MaxisSubtarget &ST);

} // End llvm namespace

#endif
