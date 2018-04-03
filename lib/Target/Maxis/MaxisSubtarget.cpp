//===-- MaxisSubtarget.cpp - Maxis Subtarget Information --------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the Maxis specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#include "MaxisSubtarget.h"
#include "Maxis.h"
#include "MaxisMachineFunction.h"
#include "MaxisRegisterInfo.h"
#include "MaxisTargetMachine.h"
#include "llvm/IR/Attributes.h"
#include "llvm/IR/Function.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "maxis-subtarget"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "MaxisGenSubtargetInfo.inc"

// FIXME: Maybe this should be on by default when Maxis16 is specified
//
static cl::opt<bool>
    Mixed16_32("maxis-mixed-16-32", cl::init(false),
               cl::desc("Allow for a mixture of Maxis16 "
                        "and Maxis32 code in a single output file"),
               cl::Hidden);

static cl::opt<bool> Maxis_Os16("maxis-os16", cl::init(false),
                               cl::desc("Compile all functions that don't use "
                                        "floating point as Maxis 16"),
                               cl::Hidden);

static cl::opt<bool> Maxis16HardFloat("maxis16-hard-float", cl::NotHidden,
                                     cl::desc("Enable maxis16 hard float."),
                                     cl::init(false));

static cl::opt<bool>
    Maxis16ConstantIslands("maxis16-constant-islands", cl::NotHidden,
                          cl::desc("Enable maxis16 constant islands."),
                          cl::init(true));

static cl::opt<bool>
    GPOpt("maxis-mgpopt", cl::Hidden,
          cl::desc("Enable gp-relative addressing of maxis small data items"));

void MaxisSubtarget::anchor() {}

MaxisSubtarget::MaxisSubtarget(const Triple &TT, StringRef CPU, StringRef FS,
                             bool little, const MaxisTargetMachine &TM,
                             unsigned StackAlignOverride)
    : MaxisGenSubtargetInfo(TT, CPU, FS), MaxisArchVersion(MaxisDefault),
      IsLittle(little), IsSoftFloat(false), IsSingleFloat(false), IsFPXX(false),
      NoABICalls(false), IsFP64bit(false), UseOddSPReg(true),
      IsNaN2008bit(false), IsGP64bit(false), HasVFPU(false), HasCnMaxis(false),
      HasMaxis3_32(false), HasMaxis3_32r2(false), HasMaxis4_32(false),
      HasMaxis4_32r2(false), HasMaxis5_32r2(false), InMaxis16Mode(false),
      InMaxis16HardFloat(Maxis16HardFloat), InMicroMaxisMode(false), HasDSP(false),
      HasDSPR2(false), HasDSPR3(false), AllowMixed16_32(Mixed16_32 | Maxis_Os16),
      Os16(Maxis_Os16), HasMSA(false), UseTCCInDIV(false), HasSym32(false),
      HasEVA(false), DisableMadd4(false), HasMT(false),
      StackAlignOverride(StackAlignOverride), TM(TM), TargetTriple(TT),
      TSInfo(), InstrInfo(MaxisInstrInfo::create(
                    initializeSubtargetDependencies(CPU, FS, TM))),
      FrameLowering(MaxisFrameLowering::create(*this)),
      TLInfo(MaxisTargetLowering::create(TM, *this)) {

  if (MaxisArchVersion == MaxisDefault)
    MaxisArchVersion = Maxis32;

  // Don't even attempt to generate code for MAXIS-I and MAXIS-V. They have not
  // been tested and currently exist for the integrated assembler only.
  if (MaxisArchVersion == Maxis1)
    report_fatal_error("Code generation for MAXIS-I is not implemented", false);
  if (MaxisArchVersion == Maxis5)
    report_fatal_error("Code generation for MAXIS-V is not implemented", false);

  // Check if Architecture and ABI are compatible.
  assert(((!isGP64bit() && isABI_O32()) ||
          (isGP64bit() && (isABI_N32() || isABI_N64()))) &&
         "Invalid  Arch & ABI pair.");

  if (hasMSA() && !isFP64bit())
    report_fatal_error("MSA requires a 64-bit FPU register file (FR=1 mode). "
                       "See -mattr=+fp64.",
                       false);

  if (!isABI_O32() && !useOddSPReg())
    report_fatal_error("-mattr=+nooddspreg requires the O32 ABI.", false);

  if (IsFPXX && (isABI_N32() || isABI_N64()))
    report_fatal_error("FPXX is not permitted for the N32/N64 ABI's.", false);

  if (hasMaxis64r6() && InMicroMaxisMode)
    report_fatal_error("microMAXIS64R6 is not supported", false);

  if (hasMaxis32r6()) {
    StringRef ISA = hasMaxis64r6() ? "MAXIS64r6" : "MAXIS32r6";

    assert(isFP64bit());
    assert(isNaN2008());
    if (hasDSP())
      report_fatal_error(ISA + " is not compatible with the DSP ASE", false);
  }

  if (NoABICalls && TM.isPositionIndependent())
    report_fatal_error("position-independent code requires '-mabicalls'");

  if (isABI_N64() && !TM.isPositionIndependent() && !hasSym32())
    NoABICalls = true;

  // Set UseSmallSection.
  UseSmallSection = GPOpt;
  if (!NoABICalls && GPOpt) {
    errs() << "warning: cannot use small-data accesses for '-mabicalls'"
           << "\n";
    UseSmallSection = false;
  }
}

bool MaxisSubtarget::isPositionIndependent() const {
  return TM.isPositionIndependent();
}

/// This overrides the PostRAScheduler bit in the SchedModel for any CPU.
bool MaxisSubtarget::enablePostRAScheduler() const { return true; }

void MaxisSubtarget::getCriticalPathRCs(RegClassVector &CriticalPathRCs) const {
  CriticalPathRCs.clear();
  CriticalPathRCs.push_back(isGP64bit() ? &Maxis::GPR64RegClass
                                        : &Maxis::GPR32RegClass);
}

CodeGenOpt::Level MaxisSubtarget::getOptLevelToEnablePostRAScheduler() const {
  return CodeGenOpt::Aggressive;
}

MaxisSubtarget &
MaxisSubtarget::initializeSubtargetDependencies(StringRef CPU, StringRef FS,
                                               const TargetMachine &TM) {
  std::string CPUName = MAXIS_MC::selectMaxisCPU(TM.getTargetTriple(), CPU);

  // Parse features string.
  ParseSubtargetFeatures(CPUName, FS);
  // Initialize scheduling itinerary for the specified CPU.
  InstrItins = getInstrItineraryForCPU(CPUName);

  if (InMaxis16Mode && !IsSoftFloat)
    InMaxis16HardFloat = true;

  if (StackAlignOverride)
    stackAlignment = StackAlignOverride;
  else if (isABI_N32() || isABI_N64())
    stackAlignment = 16;
  else {
    assert(isABI_O32() && "Unknown ABI for stack alignment!");
    stackAlignment = 8;
  }

  return *this;
}

bool MaxisSubtarget::useConstantIslands() {
  DEBUG(dbgs() << "use constant islands " << Maxis16ConstantIslands << "\n");
  return Maxis16ConstantIslands;
}

Reloc::Model MaxisSubtarget::getRelocationModel() const {
  return TM.getRelocationModel();
}

bool MaxisSubtarget::isABI_N64() const { return getABI().IsN64(); }
bool MaxisSubtarget::isABI_N32() const { return getABI().IsN32(); }
bool MaxisSubtarget::isABI_O32() const { return getABI().IsO32(); }
const MaxisABIInfo &MaxisSubtarget::getABI() const { return TM.getABI(); }
