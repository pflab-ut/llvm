//===- MaxisRegisterInfo.cpp - MAXIS Register Information -------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the MAXIS implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#include "MaxisRegisterInfo.h"
#include "MCTargetDesc/MaxisABIInfo.h"
#include "Maxis.h"
#include "MaxisMachineFunction.h"
#include "MaxisSubtarget.h"
#include "MaxisTargetMachine.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/Function.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include <cstdint>

using namespace llvm;

#define DEBUG_TYPE "maxis-reg-info"

#define GET_REGINFO_TARGET_DESC
#include "MaxisGenRegisterInfo.inc"

MaxisRegisterInfo::MaxisRegisterInfo() : MaxisGenRegisterInfo(Maxis::RA) {}

unsigned MaxisRegisterInfo::getPICCallReg() { return Maxis::T9; }

const TargetRegisterClass *
MaxisRegisterInfo::getPointerRegClass(const MachineFunction &MF,
                                     unsigned Kind) const {
  MaxisABIInfo ABI = MF.getSubtarget<MaxisSubtarget>().getABI();
  MaxisPtrClass PtrClassKind = static_cast<MaxisPtrClass>(Kind);

  switch (PtrClassKind) {
  case MaxisPtrClass::Default:
    return ABI.ArePtrs64bit() ? &Maxis::GPR64RegClass : &Maxis::GPR32RegClass;
  case MaxisPtrClass::GPR16MM:
    return &Maxis::GPRMM16RegClass;
  case MaxisPtrClass::StackPointer:
    return ABI.ArePtrs64bit() ? &Maxis::SP64RegClass : &Maxis::SP32RegClass;
  case MaxisPtrClass::GlobalPointer:
    return ABI.ArePtrs64bit() ? &Maxis::GP64RegClass : &Maxis::GP32RegClass;
  }

  llvm_unreachable("Unknown pointer kind");
}

unsigned
MaxisRegisterInfo::getRegPressureLimit(const TargetRegisterClass *RC,
                                      MachineFunction &MF) const {
  switch (RC->getID()) {
  default:
    return 0;
  case Maxis::GPR32RegClassID:
  case Maxis::GPR64RegClassID:
  case Maxis::DSPRRegClassID: {
    const TargetFrameLowering *TFI = MF.getSubtarget().getFrameLowering();
    return 28 - TFI->hasFP(MF);
  }
  case Maxis::FGR32RegClassID:
    return 32;
  case Maxis::AFGR64RegClassID:
    return 16;
  case Maxis::FGR64RegClassID:
    return 32;
  }
}

//===----------------------------------------------------------------------===//
// Callee Saved Registers methods
//===----------------------------------------------------------------------===//

/// Maxis Callee Saved Registers
const MCPhysReg *
MaxisRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  const MaxisSubtarget &Subtarget = MF->getSubtarget<MaxisSubtarget>();
  const Function &F = MF->getFunction();
  if (F.hasFnAttribute("interrupt")) {
    if (Subtarget.hasMaxis64())
      return Subtarget.hasMaxis64r6() ? CSR_Interrupt_64R6_SaveList
                                     : CSR_Interrupt_64_SaveList;
    else
      return Subtarget.hasMaxis32r6() ? CSR_Interrupt_32R6_SaveList
                                     : CSR_Interrupt_32_SaveList;
  }

  if (Subtarget.isSingleFloat())
    return CSR_SingleFloatOnly_SaveList;

  if (Subtarget.isABI_N64())
    return CSR_N64_SaveList;

  if (Subtarget.isABI_N32())
    return CSR_N32_SaveList;

  if (Subtarget.isFP64bit())
    return CSR_O32_FP64_SaveList;

  if (Subtarget.isFPXX())
    return CSR_O32_FPXX_SaveList;

  return CSR_O32_SaveList;
}

const uint32_t *
MaxisRegisterInfo::getCallPreservedMask(const MachineFunction &MF,
                                       CallingConv::ID) const {
  const MaxisSubtarget &Subtarget = MF.getSubtarget<MaxisSubtarget>();
  if (Subtarget.isSingleFloat())
    return CSR_SingleFloatOnly_RegMask;

  if (Subtarget.isABI_N64())
    return CSR_N64_RegMask;

  if (Subtarget.isABI_N32())
    return CSR_N32_RegMask;

  if (Subtarget.isFP64bit())
    return CSR_O32_FP64_RegMask;

  if (Subtarget.isFPXX())
    return CSR_O32_FPXX_RegMask;

  return CSR_O32_RegMask;
}

const uint32_t *MaxisRegisterInfo::getMaxis16RetHelperMask() {
  return CSR_Maxis16RetHelper_RegMask;
}

BitVector MaxisRegisterInfo::
getReservedRegs(const MachineFunction &MF) const {
  static const MCPhysReg ReservedGPR32[] = {
    Maxis::ZERO, Maxis::K0, Maxis::K1, Maxis::SP
  };

  static const MCPhysReg ReservedGPR64[] = {
    Maxis::ZERO_64, Maxis::K0_64, Maxis::K1_64, Maxis::SP_64
  };

  BitVector Reserved(getNumRegs());
  const MaxisSubtarget &Subtarget = MF.getSubtarget<MaxisSubtarget>();

  using RegIter = TargetRegisterClass::const_iterator;

  for (unsigned I = 0; I < array_lengthof(ReservedGPR32); ++I)
    Reserved.set(ReservedGPR32[I]);

  // Reserve registers for the NaCl sandbox.
  if (Subtarget.isTargetNaCl()) {
    Reserved.set(Maxis::T6);   // Reserved for control flow mask.
    Reserved.set(Maxis::T7);   // Reserved for memory access mask.
    Reserved.set(Maxis::T8);   // Reserved for thread pointer.
  }

  for (unsigned I = 0; I < array_lengthof(ReservedGPR64); ++I)
    Reserved.set(ReservedGPR64[I]);

  // For mno-abicalls, GP is a program invariant!
  if (!Subtarget.isABICalls()) {
    Reserved.set(Maxis::GP);
    Reserved.set(Maxis::GP_64);
  }

  if (Subtarget.isFP64bit()) {
    // Reserve all registers in AFGR64.
    for (RegIter Reg = Maxis::AFGR64RegClass.begin(),
         EReg = Maxis::AFGR64RegClass.end(); Reg != EReg; ++Reg)
      Reserved.set(*Reg);
  } else {
    // Reserve all registers in FGR64.
    for (RegIter Reg = Maxis::FGR64RegClass.begin(),
         EReg = Maxis::FGR64RegClass.end(); Reg != EReg; ++Reg)
      Reserved.set(*Reg);
  }
  // Reserve FP if this function should have a dedicated frame pointer register.
  if (Subtarget.getFrameLowering()->hasFP(MF)) {
    if (Subtarget.inMaxis16Mode())
      Reserved.set(Maxis::S0);
    else {
      Reserved.set(Maxis::FP);
      Reserved.set(Maxis::FP_64);

      // Reserve the base register if we need to both realign the stack and
      // allocate variable-sized objects at runtime. This should test the
      // same conditions as MaxisFrameLowering::hasBP().
      if (needsStackRealignment(MF) &&
          MF.getFrameInfo().hasVarSizedObjects()) {
        Reserved.set(Maxis::S7);
        Reserved.set(Maxis::S7_64);
      }
    }
  }

  // Reserve hardware registers.
  Reserved.set(Maxis::HWR29);

  // Reserve DSP control register.
  Reserved.set(Maxis::DSPPos);
  Reserved.set(Maxis::DSPSCount);
  Reserved.set(Maxis::DSPCarry);
  Reserved.set(Maxis::DSPEFI);
  Reserved.set(Maxis::DSPOutFlag);

  // Reserve MSA control registers.
  Reserved.set(Maxis::MSAIR);
  Reserved.set(Maxis::MSACSR);
  Reserved.set(Maxis::MSAAccess);
  Reserved.set(Maxis::MSASave);
  Reserved.set(Maxis::MSAModify);
  Reserved.set(Maxis::MSARequest);
  Reserved.set(Maxis::MSAMap);
  Reserved.set(Maxis::MSAUnmap);

  // Reserve RA if in maxis16 mode.
  if (Subtarget.inMaxis16Mode()) {
    const MaxisFunctionInfo *MaxisFI = MF.getInfo<MaxisFunctionInfo>();
    Reserved.set(Maxis::RA);
    Reserved.set(Maxis::RA_64);
    Reserved.set(Maxis::T0);
    Reserved.set(Maxis::T1);
    if (MF.getFunction().hasFnAttribute("saveS2") || MaxisFI->hasSaveS2())
      Reserved.set(Maxis::S2);
  }

  // Reserve GP if small section is used.
  if (Subtarget.useSmallSection()) {
    Reserved.set(Maxis::GP);
    Reserved.set(Maxis::GP_64);
  }

  if (Subtarget.isABI_O32() && !Subtarget.useOddSPReg()) {
    for (const auto &Reg : Maxis::OddSPRegClass)
      Reserved.set(Reg);
  }

  return Reserved;
}

bool
MaxisRegisterInfo::requiresRegisterScavenging(const MachineFunction &MF) const {
  return true;
}

bool
MaxisRegisterInfo::trackLivenessAfterRegAlloc(const MachineFunction &MF) const {
  return true;
}

// FrameIndex represent objects inside a abstract stack.
// We must replace FrameIndex with an stack/frame pointer
// direct reference.
void MaxisRegisterInfo::
eliminateFrameIndex(MachineBasicBlock::iterator II, int SPAdj,
                    unsigned FIOperandNum, RegScavenger *RS) const {
  MachineInstr &MI = *II;
  MachineFunction &MF = *MI.getParent()->getParent();

  DEBUG(errs() << "\nFunction : " << MF.getName() << "\n";
        errs() << "<--------->\n" << MI);

  int FrameIndex = MI.getOperand(FIOperandNum).getIndex();
  uint64_t stackSize = MF.getFrameInfo().getStackSize();
  int64_t spOffset = MF.getFrameInfo().getObjectOffset(FrameIndex);

  DEBUG(errs() << "FrameIndex : " << FrameIndex << "\n"
               << "spOffset   : " << spOffset << "\n"
               << "stackSize  : " << stackSize << "\n"
               << "alignment  : "
               << MF.getFrameInfo().getObjectAlignment(FrameIndex) << "\n");

  eliminateFI(MI, FIOperandNum, FrameIndex, stackSize, spOffset);
}

unsigned MaxisRegisterInfo::
getFrameRegister(const MachineFunction &MF) const {
  const MaxisSubtarget &Subtarget = MF.getSubtarget<MaxisSubtarget>();
  const TargetFrameLowering *TFI = Subtarget.getFrameLowering();
  bool IsN64 =
      static_cast<const MaxisTargetMachine &>(MF.getTarget()).getABI().IsN64();

  if (Subtarget.inMaxis16Mode())
    return TFI->hasFP(MF) ? Maxis::S0 : Maxis::SP;
  else
    return TFI->hasFP(MF) ? (IsN64 ? Maxis::FP_64 : Maxis::FP) :
                            (IsN64 ? Maxis::SP_64 : Maxis::SP);
}

bool MaxisRegisterInfo::canRealignStack(const MachineFunction &MF) const {
  // Avoid realigning functions that explicitly do not want to be realigned.
  // Normally, we should report an error when a function should be dynamically
  // realigned but also has the attribute no-realign-stack. Unfortunately,
  // with this attribute, MachineFrameInfo clamps each new object's alignment
  // to that of the stack's alignment as specified by the ABI. As a result,
  // the information of whether we have objects with larger alignment
  // requirement than the stack's alignment is already lost at this point.
  if (!TargetRegisterInfo::canRealignStack(MF))
    return false;

  const MaxisSubtarget &Subtarget = MF.getSubtarget<MaxisSubtarget>();
  unsigned FP = Subtarget.isGP32bit() ? Maxis::FP : Maxis::FP_64;
  unsigned BP = Subtarget.isGP32bit() ? Maxis::S7 : Maxis::S7_64;

  // Support dynamic stack realignment only for targets with standard encoding.
  if (!Subtarget.hasStandardEncoding())
    return false;

  // We can't perform dynamic stack realignment if we can't reserve the
  // frame pointer register.
  if (!MF.getRegInfo().canReserveReg(FP))
    return false;

  // We can realign the stack if we know the maximum call frame size and we
  // don't have variable sized objects.
  if (Subtarget.getFrameLowering()->hasReservedCallFrame(MF))
    return true;

  // We have to reserve the base pointer register in the presence of variable
  // sized objects.
  return MF.getRegInfo().canReserveReg(BP);
}
