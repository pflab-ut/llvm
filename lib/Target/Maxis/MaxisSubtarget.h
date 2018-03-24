//===-- MaxisSubtarget.h - Define Subtarget for the Maxis ---------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the Maxis specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MAXIS_MAXISSUBTARGET_H
#define LLVM_LIB_TARGET_MAXIS_MAXISSUBTARGET_H

#include "MCTargetDesc/MaxisABIInfo.h"
#include "MaxisFrameLowering.h"
#include "MaxisISelLowering.h"
#include "MaxisInstrInfo.h"
#include "llvm/CodeGen/SelectionDAGTargetInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/MC/MCInstrItineraries.h"
#include "llvm/Support/ErrorHandling.h"
#include <string>

#define GET_SUBTARGETINFO_HEADER
#include "MaxisGenSubtargetInfo.inc"

namespace llvm {
class StringRef;

class MaxisTargetMachine;

class MaxisSubtarget : public MaxisGenSubtargetInfo {
  virtual void anchor();

  enum MaxisArchEnum {
    MaxisDefault,
    Maxis1, Maxis2, Maxis32, Maxis32r2, Maxis32r3, Maxis32r5, Maxis32r6, Maxis32Max,
    Maxis3, Maxis4, Maxis5, Maxis64, Maxis64r2, Maxis64r3, Maxis64r5, Maxis64r6
  };

  enum class CPU { P5600 };

  // Maxis architecture version
  MaxisArchEnum MaxisArchVersion;

  // Processor implementation (unused but required to exist by
  // tablegen-erated code).
  CPU ProcImpl;

  // IsLittle - The target is Little Endian
  bool IsLittle;

  // IsSoftFloat - The target does not support any floating point instructions.
  bool IsSoftFloat;

  // IsSingleFloat - The target only supports single precision float
  // point operations. This enable the target to use all 32 32-bit
  // floating point registers instead of only using even ones.
  bool IsSingleFloat;

  // IsFPXX - MAXIS O32 modeless ABI.
  bool IsFPXX;

  // NoABICalls - Disable SVR4-style position-independent code.
  bool NoABICalls;

  // IsFP64bit - The target processor has 64-bit floating point registers.
  bool IsFP64bit;

  /// Are odd single-precision registers permitted?
  /// This corresponds to -modd-spreg and -mno-odd-spreg
  bool UseOddSPReg;

  // IsNan2008 - IEEE 754-2008 NaN encoding.
  bool IsNaN2008bit;

  // IsGP64bit - General-purpose registers are 64 bits wide
  bool IsGP64bit;

  // IsPTR64bit - Pointers are 64 bit wide
  bool IsPTR64bit;

  // HasVFPU - Processor has a vector floating point unit.
  bool HasVFPU;

  // CPU supports cnMAXIS (Cavium Networks Octeon CPU).
  bool HasCnMaxis;

  // isLinux - Target system is Linux. Is false we consider ELFOS for now.
  bool IsLinux;

  // UseSmallSection - Small section is used.
  bool UseSmallSection;

  /// Features related to the presence of specific instructions.

  // HasMaxis3_32 - The subset of MAXIS-III instructions added to MAXIS32
  bool HasMaxis3_32;

  // HasMaxis3_32r2 - The subset of MAXIS-III instructions added to MAXIS32r2
  bool HasMaxis3_32r2;

  // HasMaxis4_32 - Has the subset of MAXIS-IV present in MAXIS32
  bool HasMaxis4_32;

  // HasMaxis4_32r2 - Has the subset of MAXIS-IV present in MAXIS32r2
  bool HasMaxis4_32r2;

  // HasMaxis5_32r2 - Has the subset of MAXIS-V present in MAXIS32r2
  bool HasMaxis5_32r2;

  // InMaxis16 -- can process Maxis16 instructions
  bool InMaxis16Mode;

  // Maxis16 hard float
  bool InMaxis16HardFloat;

  // InMicroMaxis -- can process MicroMaxis instructions
  bool InMicroMaxisMode;

  // HasDSP, HasDSPR2, HasDSPR3 -- supports DSP ASE.
  bool HasDSP, HasDSPR2, HasDSPR3;

  // Allow mixed Maxis16 and Maxis32 in one source file
  bool AllowMixed16_32;

  // Optimize for space by compiling all functions as Maxis 16 unless
  // it needs floating point. Functions needing floating point are
  // compiled as Maxis32
  bool Os16;

  // HasMSA -- supports MSA ASE.
  bool HasMSA;

  // UseTCCInDIV -- Enables the use of trapping in the assembler.
  bool UseTCCInDIV;

  // Sym32 -- On Maxis64 symbols are 32 bits.
  bool HasSym32;

  // HasEVA -- supports EVA ASE.
  bool HasEVA;
 
  // nomadd4 - disables generation of 4-operand madd.s, madd.d and
  // related instructions.
  bool DisableMadd4;

  // HasMT -- support MT ASE.
  bool HasMT;

  // Disable use of the `jal` instruction.
  bool UseLongCalls = false;

  /// The minimum alignment known to hold of the stack frame on
  /// entry to the function and which must be maintained by every function.
  unsigned stackAlignment;

  /// The overridden stack alignment.
  unsigned StackAlignOverride;

  InstrItineraryData InstrItins;

  // We can override the determination of whether we are in maxis16 mode
  // as from the command line
  enum {NoOverride, Maxis16Override, NoMaxis16Override} OverrideMode;

  const MaxisTargetMachine &TM;

  Triple TargetTriple;

  const SelectionDAGTargetInfo TSInfo;
  std::unique_ptr<const MaxisInstrInfo> InstrInfo;
  std::unique_ptr<const MaxisFrameLowering> FrameLowering;
  std::unique_ptr<const MaxisTargetLowering> TLInfo;

public:
  bool isPositionIndependent() const;
  /// This overrides the PostRAScheduler bit in the SchedModel for each CPU.
  bool enablePostRAScheduler() const override;
  void getCriticalPathRCs(RegClassVector &CriticalPathRCs) const override;
  CodeGenOpt::Level getOptLevelToEnablePostRAScheduler() const override;

  bool isABI_N64() const;
  bool isABI_N32() const;
  bool isABI_O32() const;
  const MaxisABIInfo &getABI() const;
  bool isABI_FPXX() const { return isABI_O32() && IsFPXX; }

  /// This constructor initializes the data members to match that
  /// of the specified triple.
  MaxisSubtarget(const Triple &TT, StringRef CPU, StringRef FS, bool little,
                const MaxisTargetMachine &TM, unsigned StackAlignOverride);

  /// ParseSubtargetFeatures - Parses features string setting specified
  /// subtarget options.  Definition of function is auto generated by tblgen.
  void ParseSubtargetFeatures(StringRef CPU, StringRef FS);

  bool hasMaxis1() const { return MaxisArchVersion >= Maxis1; }
  bool hasMaxis2() const { return MaxisArchVersion >= Maxis2; }
  bool hasMaxis3() const { return MaxisArchVersion >= Maxis3; }
  bool hasMaxis4() const { return MaxisArchVersion >= Maxis4; }
  bool hasMaxis5() const { return MaxisArchVersion >= Maxis5; }
  bool hasMaxis4_32() const { return HasMaxis4_32; }
  bool hasMaxis4_32r2() const { return HasMaxis4_32r2; }
  bool hasMaxis32() const {
    return (MaxisArchVersion >= Maxis32 && MaxisArchVersion < Maxis32Max) ||
           hasMaxis64();
  }
  bool hasMaxis32r2() const {
    return (MaxisArchVersion >= Maxis32r2 && MaxisArchVersion < Maxis32Max) ||
           hasMaxis64r2();
  }
  bool hasMaxis32r3() const {
    return (MaxisArchVersion >= Maxis32r3 && MaxisArchVersion < Maxis32Max) ||
           hasMaxis64r2();
  }
  bool hasMaxis32r5() const {
    return (MaxisArchVersion >= Maxis32r5 && MaxisArchVersion < Maxis32Max) ||
           hasMaxis64r5();
  }
  bool hasMaxis32r6() const {
    return (MaxisArchVersion >= Maxis32r6 && MaxisArchVersion < Maxis32Max) ||
           hasMaxis64r6();
  }
  bool hasMaxis64() const { return MaxisArchVersion >= Maxis64; }
  bool hasMaxis64r2() const { return MaxisArchVersion >= Maxis64r2; }
  bool hasMaxis64r3() const { return MaxisArchVersion >= Maxis64r3; }
  bool hasMaxis64r5() const { return MaxisArchVersion >= Maxis64r5; }
  bool hasMaxis64r6() const { return MaxisArchVersion >= Maxis64r6; }

  bool hasCnMaxis() const { return HasCnMaxis; }

  bool isLittle() const { return IsLittle; }
  bool isABICalls() const { return !NoABICalls; }
  bool isFPXX() const { return IsFPXX; }
  bool isFP64bit() const { return IsFP64bit; }
  bool useOddSPReg() const { return UseOddSPReg; }
  bool noOddSPReg() const { return !UseOddSPReg; }
  bool isNaN2008() const { return IsNaN2008bit; }
  bool isGP64bit() const { return IsGP64bit; }
  bool isGP32bit() const { return !IsGP64bit; }
  unsigned getGPRSizeInBytes() const { return isGP64bit() ? 8 : 4; }
  bool isPTR64bit() const { return IsPTR64bit; }
  bool isPTR32bit() const { return !IsPTR64bit; }
  bool hasSym32() const {
    return (HasSym32 && isABI_N64()) || isABI_N32() || isABI_O32();
  }
  bool isSingleFloat() const { return IsSingleFloat; }
  bool isTargetELF() const { return TargetTriple.isOSBinFormatELF(); }
  bool hasVFPU() const { return HasVFPU; }
  bool inMaxis16Mode() const { return InMaxis16Mode; }
  bool inMaxis16ModeDefault() const {
    return InMaxis16Mode;
  }
  // Hard float for maxis16 means essentially to compile as soft float
  // but to use a runtime library for soft float that is written with
  // native maxis32 floating point instructions (those runtime routines
  // run in maxis32 hard float mode).
  bool inMaxis16HardFloat() const {
    return inMaxis16Mode() && InMaxis16HardFloat;
  }
  bool inMicroMaxisMode() const { return InMicroMaxisMode; }
  bool inMicroMaxis32r6Mode() const { return InMicroMaxisMode && hasMaxis32r6(); }
  bool hasDSP() const { return HasDSP; }
  bool hasDSPR2() const { return HasDSPR2; }
  bool hasDSPR3() const { return HasDSPR3; }
  bool hasMSA() const { return HasMSA; }
  bool disableMadd4() const { return DisableMadd4; }
  bool hasEVA() const { return HasEVA; }
  bool hasMT() const { return HasMT; }
  bool useSmallSection() const { return UseSmallSection; }

  bool hasStandardEncoding() const { return !inMaxis16Mode(); }

  bool useSoftFloat() const { return IsSoftFloat; }

  bool useLongCalls() const { return UseLongCalls; }

  bool enableLongBranchPass() const {
    return hasStandardEncoding() || allowMixed16_32();
  }

  /// Features related to the presence of specific instructions.
  bool hasExtractInsert() const { return !inMaxis16Mode() && hasMaxis32r2(); }
  bool hasMTHC1() const { return hasMaxis32r2(); }

  bool allowMixed16_32() const { return inMaxis16ModeDefault() |
                                        AllowMixed16_32; }

  bool os16() const { return Os16; }

  bool isTargetNaCl() const { return TargetTriple.isOSNaCl(); }

  bool isXRaySupported() const override { return true; }

  // for now constant islands are on for the whole compilation unit but we only
  // really use them if in addition we are in maxis16 mode
  static bool useConstantIslands();

  unsigned getStackAlignment() const { return stackAlignment; }

  // Grab relocation model
  Reloc::Model getRelocationModel() const;

  MaxisSubtarget &initializeSubtargetDependencies(StringRef CPU, StringRef FS,
                                                 const TargetMachine &TM);

  /// Does the system support unaligned memory access.
  ///
  /// MAXIS32r6/MAXIS64r6 require full unaligned access support but does not
  /// specify which component of the system provides it. Hardware, software, and
  /// hybrid implementations are all valid.
  bool systemSupportsUnalignedAccess() const { return hasMaxis32r6(); }

  // Set helper classes
  void setHelperClassesMaxis16();
  void setHelperClassesMaxisSE();

  const SelectionDAGTargetInfo *getSelectionDAGInfo() const override {
    return &TSInfo;
  }
  const MaxisInstrInfo *getInstrInfo() const override { return InstrInfo.get(); }
  const TargetFrameLowering *getFrameLowering() const override {
    return FrameLowering.get();
  }
  const MaxisRegisterInfo *getRegisterInfo() const override {
    return &InstrInfo->getRegisterInfo();
  }
  const MaxisTargetLowering *getTargetLowering() const override {
    return TLInfo.get();
  }
  const InstrItineraryData *getInstrItineraryData() const override {
    return &InstrItins;
  }
};
} // End llvm namespace

#endif
