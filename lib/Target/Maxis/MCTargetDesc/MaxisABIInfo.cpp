//===---- MaxisABIInfo.cpp - Information about MAXIS ABI's ------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MaxisABIInfo.h"
#include "MaxisRegisterInfo.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/MC/MCTargetOptions.h"

using namespace llvm;

namespace {
static const MCPhysReg O32IntRegs[4] = {Maxis::A0, Maxis::A1, Maxis::A2, Maxis::A3};

static const MCPhysReg Maxis64IntRegs[8] = {
    Maxis::A0_64, Maxis::A1_64, Maxis::A2_64, Maxis::A3_64,
    Maxis::T0_64, Maxis::T1_64, Maxis::T2_64, Maxis::T3_64};
}

ArrayRef<MCPhysReg> MaxisABIInfo::GetByValArgRegs() const {
  if (IsO32())
    return makeArrayRef(O32IntRegs);
  if (IsN32() || IsN64())
    return makeArrayRef(Maxis64IntRegs);
  llvm_unreachable("Unhandled ABI");
}

ArrayRef<MCPhysReg> MaxisABIInfo::GetVarArgRegs() const {
  if (IsO32())
    return makeArrayRef(O32IntRegs);
  if (IsN32() || IsN64())
    return makeArrayRef(Maxis64IntRegs);
  llvm_unreachable("Unhandled ABI");
}

unsigned MaxisABIInfo::GetCalleeAllocdArgSizeInBytes(CallingConv::ID CC) const {
  if (IsO32())
    return CC != CallingConv::Fast ? 16 : 0;
  if (IsN32() || IsN64())
    return 0;
  llvm_unreachable("Unhandled ABI");
}

MaxisABIInfo MaxisABIInfo::computeTargetABI(const Triple &TT, StringRef CPU,
                                          const MCTargetOptions &Options) {
  if (Options.getABIName().startswith("o32"))
    return MaxisABIInfo::O32();
  if (Options.getABIName().startswith("n32"))
    return MaxisABIInfo::N32();
  if (Options.getABIName().startswith("n64"))
    return MaxisABIInfo::N64();
  assert(Options.getABIName().empty() && "Unknown ABI option for MAXIS");

  if (TT.getArch() == Triple::maxis64 || TT.getArch() == Triple::maxis64el)
    return MaxisABIInfo::N64();
  return MaxisABIInfo::O32();
}

unsigned MaxisABIInfo::GetStackPtr() const {
  return ArePtrs64bit() ? Maxis::SP_64 : Maxis::SP;
}

unsigned MaxisABIInfo::GetFramePtr() const {
  return ArePtrs64bit() ? Maxis::FP_64 : Maxis::FP;
}

unsigned MaxisABIInfo::GetBasePtr() const {
  return ArePtrs64bit() ? Maxis::S7_64 : Maxis::S7;
}

unsigned MaxisABIInfo::GetGlobalPtr() const {
  return ArePtrs64bit() ? Maxis::GP_64 : Maxis::GP;
}

unsigned MaxisABIInfo::GetNullPtr() const {
  return ArePtrs64bit() ? Maxis::ZERO_64 : Maxis::ZERO;
}

unsigned MaxisABIInfo::GetZeroReg() const {
  return AreGprs64bit() ? Maxis::ZERO_64 : Maxis::ZERO;
}

unsigned MaxisABIInfo::GetPtrAddOp() const {
  return ArePtrs64bit() ? Maxis::DADDu : Maxis::ADD;
}

unsigned MaxisABIInfo::GetPtrAddiOp() const {
  return ArePtrs64bit() ? Maxis::DADDi : Maxis::ADDi;
}

unsigned MaxisABIInfo::GetPtrSubOp() const {
  return ArePtrs64bit() ? Maxis::DSUBu : Maxis::SUB;
}

unsigned MaxisABIInfo::GetPtrAndOp() const {
  return ArePtrs64bit() ? Maxis::AND64 : Maxis::AND;
}

unsigned MaxisABIInfo::GetGPRMoveOp() const {
  return ArePtrs64bit() ? Maxis::OR64 : Maxis::OR;
}

unsigned MaxisABIInfo::GetEhDataReg(unsigned I) const {
  static const unsigned EhDataReg[] = {
    Maxis::A0, Maxis::A1, Maxis::A2, Maxis::A3
  };
  static const unsigned EhDataReg64[] = {
    Maxis::A0_64, Maxis::A1_64, Maxis::A2_64, Maxis::A3_64
  };

  return IsN64() ? EhDataReg64[I] : EhDataReg[I];
}

