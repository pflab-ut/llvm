//===-- MaxisMachineFunctionInfo.cpp - Private data used for Maxis ----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MaxisMachineFunction.h"
#include "MCTargetDesc/MaxisABIInfo.h"
#include "MaxisSubtarget.h"
#include "MaxisTargetMachine.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/PseudoSourceValue.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/Support/CommandLine.h"

using namespace llvm;

static cl::opt<bool>
FixGlobalBaseReg("maxis-fix-global-base-reg", cl::Hidden, cl::init(true),
                 cl::desc("Always use $gp as the global base register."));

MaxisFunctionInfo::~MaxisFunctionInfo() = default;

bool MaxisFunctionInfo::globalBaseRegSet() const {
  return GlobalBaseReg;
}

unsigned MaxisFunctionInfo::getGlobalBaseReg() {
  // Return if it has already been initialized.
  if (GlobalBaseReg)
    return GlobalBaseReg;

  MaxisSubtarget const &STI =
      static_cast<const MaxisSubtarget &>(MF.getSubtarget());

  const TargetRegisterClass *RC =
      STI.inMaxis16Mode()
          ? &Maxis::CPU16RegsRegClass
          : STI.inMicroMaxisMode()
                ? &Maxis::GPRMM16RegClass
                : static_cast<const MaxisTargetMachine &>(MF.getTarget())
                          .getABI()
                          .IsN64()
                      ? &Maxis::GPR64RegClass
                      : &Maxis::GPR32RegClass;
  return GlobalBaseReg = MF.getRegInfo().createVirtualRegister(RC);
}

void MaxisFunctionInfo::createEhDataRegsFI() {
  const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();
  for (int I = 0; I < 4; ++I) {
    const TargetRegisterClass &RC =
        static_cast<const MaxisTargetMachine &>(MF.getTarget()).getABI().IsN64()
            ? Maxis::GPR64RegClass
            : Maxis::GPR32RegClass;

    EhDataRegFI[I] = MF.getFrameInfo().CreateStackObject(TRI.getSpillSize(RC),
        TRI.getSpillAlignment(RC), false);
  }
}

void MaxisFunctionInfo::createISRRegFI() {
  // ISRs require spill slots for Status & ErrorPC Coprocessor 0 registers.
  // The current implementation only supports Maxis32r2+ not Maxis64rX. Status
  // is always 32 bits, ErrorPC is 32 or 64 bits dependent on architecture,
  // however Maxis32r2+ is the supported architecture.
  const TargetRegisterClass &RC = Maxis::GPR32RegClass;
  const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();

  for (int I = 0; I < 2; ++I)
    ISRDataRegFI[I] = MF.getFrameInfo().CreateStackObject(
        TRI.getSpillSize(RC), TRI.getSpillAlignment(RC), false);
}

bool MaxisFunctionInfo::isEhDataRegFI(int FI) const {
  return CallsEhReturn && (FI == EhDataRegFI[0] || FI == EhDataRegFI[1]
                        || FI == EhDataRegFI[2] || FI == EhDataRegFI[3]);
}

bool MaxisFunctionInfo::isISRRegFI(int FI) const {
  return IsISR && (FI == ISRDataRegFI[0] || FI == ISRDataRegFI[1]);
}
MachinePointerInfo MaxisFunctionInfo::callPtrInfo(const char *ES) {
  return MachinePointerInfo(MF.getPSVManager().getExternalSymbolCallEntry(ES));
}

MachinePointerInfo MaxisFunctionInfo::callPtrInfo(const GlobalValue *GV) {
  return MachinePointerInfo(MF.getPSVManager().getGlobalValueCallEntry(GV));
}

int MaxisFunctionInfo::getMoveF64ViaSpillFI(const TargetRegisterClass *RC) {
  const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();
  if (MoveF64ViaSpillFI == -1) {
    MoveF64ViaSpillFI = MF.getFrameInfo().CreateStackObject(
        TRI.getSpillSize(*RC), TRI.getSpillAlignment(*RC), false);
  }
  return MoveF64ViaSpillFI;
}

void MaxisFunctionInfo::anchor() {}
