//===--- Maxis.cpp - Tools Implementations -----------------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "Maxis.h"
#include "ToolChains/CommonArgs.h"
#include "clang/Driver/Driver.h"
#include "clang/Driver/DriverDiagnostic.h"
#include "clang/Driver/Options.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/Option/ArgList.h"

using namespace clang::driver;
using namespace clang::driver::tools;
using namespace clang;
using namespace llvm::opt;

bool tools::isMaxisArch(llvm::Triple::ArchType Arch) {
  return Arch == llvm::Triple::maxis || Arch == llvm::Triple::maxisel ||
         Arch == llvm::Triple::maxis64 || Arch == llvm::Triple::maxis64el;
}

// Get CPU and ABI names. They are not independent
// so we have to calculate them together.
void maxis::getMaxisCPUAndABI(const ArgList &Args, const llvm::Triple &Triple,
                            StringRef &CPUName, StringRef &ABIName) {
  const char *DefMaxis32CPU = "maxis32r2";
  const char *DefMaxis64CPU = "maxis64r2";

  // MAXIS32r6 is the default for maxis(el)?-img-linux-gnu and MAXIS64r6 is the
  // default for maxis64(el)?-img-linux-gnu.
  if (Triple.getVendor() == llvm::Triple::ImaginationTechnologies &&
      Triple.isGNUEnvironment()) {
    DefMaxis32CPU = "maxis32r6";
    DefMaxis64CPU = "maxis64r6";
  }

  // MAXIS64r6 is the default for Android MAXIS64 (maxis64el-linux-android).
  if (Triple.isAndroid()) {
    DefMaxis32CPU = "maxis32";
    DefMaxis64CPU = "maxis64r6";
  }

  // MAXIS3 is the default for maxis64*-unknown-openbsd.
  if (Triple.getOS() == llvm::Triple::OpenBSD)
    DefMaxis64CPU = "maxis3";

  if (Arg *A = Args.getLastArg(clang::driver::options::OPT_march_EQ,
                               options::OPT_mcpu_EQ))
    CPUName = A->getValue();

  if (Arg *A = Args.getLastArg(options::OPT_mabi_EQ)) {
    ABIName = A->getValue();
    // Convert a GNU style Maxis ABI name to the name
    // accepted by LLVM Maxis backend.
    ABIName = llvm::StringSwitch<llvm::StringRef>(ABIName)
                  .Case("32", "o32")
                  .Case("64", "n64")
                  .Default(ABIName);
  }

  // Setup default CPU and ABI names.
  if (CPUName.empty() && ABIName.empty()) {
    switch (Triple.getArch()) {
    default:
      llvm_unreachable("Unexpected triple arch name");
    case llvm::Triple::maxis:
    case llvm::Triple::maxisel:
      CPUName = DefMaxis32CPU;
      break;
    case llvm::Triple::maxis64:
    case llvm::Triple::maxis64el:
      CPUName = DefMaxis64CPU;
      break;
    }
  }

  if (ABIName.empty() &&
      (Triple.getVendor() == llvm::Triple::MaxisTechnologies ||
       Triple.getVendor() == llvm::Triple::ImaginationTechnologies)) {
    ABIName = llvm::StringSwitch<const char *>(CPUName)
                  .Case("maxis1", "o32")
                  .Case("maxis2", "o32")
                  .Case("maxis3", "n64")
                  .Case("maxis4", "n64")
                  .Case("maxis5", "n64")
                  .Case("maxis32", "o32")
                  .Case("maxis32r2", "o32")
                  .Case("maxis32r3", "o32")
                  .Case("maxis32r5", "o32")
                  .Case("maxis32r6", "o32")
                  .Case("maxis64", "n64")
                  .Case("maxis64r2", "n64")
                  .Case("maxis64r3", "n64")
                  .Case("maxis64r5", "n64")
                  .Case("maxis64r6", "n64")
                  .Case("octeon", "n64")
                  .Case("p5600", "o32")
                  .Default("");
  }

  if (ABIName.empty()) {
    // Deduce ABI name from the target triple.
    if (Triple.getArch() == llvm::Triple::maxis ||
        Triple.getArch() == llvm::Triple::maxisel)
      ABIName = "o32";
    else
      ABIName = "n64";
  }

  if (CPUName.empty()) {
    // Deduce CPU name from ABI name.
    CPUName = llvm::StringSwitch<const char *>(ABIName)
                  .Case("o32", DefMaxis32CPU)
                  .Cases("n32", "n64", DefMaxis64CPU)
                  .Default("");
  }

  // FIXME: Warn on inconsistent use of -march and -mabi.
}

std::string maxis::getMaxisABILibSuffix(const ArgList &Args,
                                      const llvm::Triple &Triple) {
  StringRef CPUName, ABIName;
  tools::maxis::getMaxisCPUAndABI(Args, Triple, CPUName, ABIName);
  return llvm::StringSwitch<std::string>(ABIName)
      .Case("o32", "")
      .Case("n32", "32")
      .Case("n64", "64");
}

// Convert ABI name to the GNU tools acceptable variant.
StringRef maxis::getGnuCompatibleMaxisABIName(StringRef ABI) {
  return llvm::StringSwitch<llvm::StringRef>(ABI)
      .Case("o32", "32")
      .Case("n64", "64")
      .Default(ABI);
}

// Select the MAXIS float ABI as determined by -msoft-float, -mhard-float,
// and -mfloat-abi=.
maxis::FloatABI maxis::getMaxisFloatABI(const Driver &D, const ArgList &Args) {
  maxis::FloatABI ABI = maxis::FloatABI::Invalid;
  if (Arg *A =
          Args.getLastArg(options::OPT_msoft_float, options::OPT_mhard_float,
                          options::OPT_mfloat_abi_EQ)) {
    if (A->getOption().matches(options::OPT_msoft_float))
      ABI = maxis::FloatABI::Soft;
    else if (A->getOption().matches(options::OPT_mhard_float))
      ABI = maxis::FloatABI::Hard;
    else {
      ABI = llvm::StringSwitch<maxis::FloatABI>(A->getValue())
                .Case("soft", maxis::FloatABI::Soft)
                .Case("hard", maxis::FloatABI::Hard)
                .Default(maxis::FloatABI::Invalid);
      if (ABI == maxis::FloatABI::Invalid && !StringRef(A->getValue()).empty()) {
        D.Diag(clang::diag::err_drv_invalid_mfloat_abi) << A->getAsString(Args);
        ABI = maxis::FloatABI::Hard;
      }
    }
  }

  // If unspecified, choose the default based on the platform.
  if (ABI == maxis::FloatABI::Invalid) {
    // Assume "hard", because it's a default value used by gcc.
    // When we start to recognize specific target MAXIS processors,
    // we will be able to select the default more correctly.
    ABI = maxis::FloatABI::Hard;
  }

  assert(ABI != maxis::FloatABI::Invalid && "must select an ABI");
  return ABI;
}

void maxis::getMAXISTargetFeatures(const Driver &D, const llvm::Triple &Triple,
                                 const ArgList &Args,
                                 std::vector<StringRef> &Features) {
  StringRef CPUName;
  StringRef ABIName;
  getMaxisCPUAndABI(Args, Triple, CPUName, ABIName);
  ABIName = getGnuCompatibleMaxisABIName(ABIName);

  // Historically, PIC code for MAXIS was associated with -mabicalls, a.k.a
  // SVR4 abicalls. Static code does not use SVR4 calling sequences. An ABI
  // extension was developed by Richard Sandiford & Code Sourcery to support
  // static code calling PIC code (CPIC). For O32 and N32 this means we have
  // several combinations of PIC/static and abicalls. Pure static, static
  // with the CPIC extension, and pure PIC code.

  // At final link time, O32 and N32 with CPIC will have another section
  // added to the binary which contains the stub functions to perform
  // any fixups required for PIC code.

  // For N64, the situation is more regular: code can either be static
  // (non-abicalls) or PIC (abicalls). GCC has traditionally picked PIC code
  // code for N64. Since Clang has already built the relocation model portion
  // of the commandline, we pick add +noabicalls feature in the N64 static
  // case.

  // The is another case to be accounted for: -msym32, which enforces that all
  // symbols have 32 bits in size. In this case, N64 can in theory use CPIC
  // but it is unsupported.

  // The combinations for N64 are:
  // a) Static without abicalls and 64bit symbols.
  // b) Static with abicalls and 32bit symbols.
  // c) PIC with abicalls and 64bit symbols.

  // For case (a) we need to add +noabicalls for N64.

  bool IsN64 = ABIName == "64";
  bool NonPIC = false;

  Arg *LastPICArg = Args.getLastArg(options::OPT_fPIC, options::OPT_fno_PIC,
                                    options::OPT_fpic, options::OPT_fno_pic,
                                    options::OPT_fPIE, options::OPT_fno_PIE,
                                    options::OPT_fpie, options::OPT_fno_pie);
  if (LastPICArg) {
    Option O = LastPICArg->getOption();
    NonPIC =
        (O.matches(options::OPT_fno_PIC) || O.matches(options::OPT_fno_pic) ||
         O.matches(options::OPT_fno_PIE) || O.matches(options::OPT_fno_pie));
  }

  bool UseAbiCalls = false;

  Arg *ABICallsArg =
      Args.getLastArg(options::OPT_mabicalls, options::OPT_mno_abicalls);
  UseAbiCalls =
      !ABICallsArg || ABICallsArg->getOption().matches(options::OPT_mabicalls);

  if (UseAbiCalls && IsN64 && NonPIC) {
    D.Diag(diag::warn_drv_unsupported_abicalls);
    UseAbiCalls = false;
  }

  if (!UseAbiCalls)
    Features.push_back("+noabicalls");
  else
    Features.push_back("-noabicalls");

  if (Arg *A = Args.getLastArg(options::OPT_mlong_calls,
                               options::OPT_mno_long_calls)) {
    if (A->getOption().matches(options::OPT_mno_long_calls))
      Features.push_back("-long-calls");
    else if (!UseAbiCalls)
      Features.push_back("+long-calls");
    else
      D.Diag(diag::warn_drv_unsupported_longcalls) << (ABICallsArg ? 0 : 1);
  }

  maxis::FloatABI FloatABI = maxis::getMaxisFloatABI(D, Args);
  if (FloatABI == maxis::FloatABI::Soft) {
    // FIXME: Note, this is a hack. We need to pass the selected float
    // mode to the MaxisTargetInfoBase to define appropriate macros there.
    // Now it is the only method.
    Features.push_back("+soft-float");
  }

  if (Arg *A = Args.getLastArg(options::OPT_mnan_EQ)) {
    StringRef Val = StringRef(A->getValue());
    if (Val == "2008") {
      if (maxis::getIEEE754Standard(CPUName) & maxis::Std2008)
        Features.push_back("+nan2008");
      else {
        Features.push_back("-nan2008");
        D.Diag(diag::warn_target_unsupported_nan2008) << CPUName;
      }
    } else if (Val == "legacy") {
      if (maxis::getIEEE754Standard(CPUName) & maxis::Legacy)
        Features.push_back("-nan2008");
      else {
        Features.push_back("+nan2008");
        D.Diag(diag::warn_target_unsupported_nanlegacy) << CPUName;
      }
    } else
      D.Diag(diag::err_drv_unsupported_option_argument)
          << A->getOption().getName() << Val;
  }

  if (Arg *A = Args.getLastArg(options::OPT_mabs_EQ)) {
    StringRef Val = StringRef(A->getValue());
    if (Val == "2008") {
      if (maxis::getIEEE754Standard(CPUName) & maxis::Std2008) {
        Features.push_back("+abs2008");
      } else {
        Features.push_back("-abs2008");
        D.Diag(diag::warn_target_unsupported_abs2008) << CPUName;
      }
    } else if (Val == "legacy") {
      if (maxis::getIEEE754Standard(CPUName) & maxis::Legacy) {
        Features.push_back("-abs2008");
      } else {
        Features.push_back("+abs2008");
        D.Diag(diag::warn_target_unsupported_abslegacy) << CPUName;
      }
    } else {
      D.Diag(diag::err_drv_unsupported_option_argument)
          << A->getOption().getName() << Val;
    }
  }

  AddTargetFeature(Args, Features, options::OPT_msingle_float,
                   options::OPT_mdouble_float, "single-float");
  AddTargetFeature(Args, Features, options::OPT_maxis16, options::OPT_mno_maxis16,
                   "maxis16");
  AddTargetFeature(Args, Features, options::OPT_mmicromaxis,
                   options::OPT_mno_micromaxis, "micromaxis");
  AddTargetFeature(Args, Features, options::OPT_mdsp, options::OPT_mno_dsp,
                   "dsp");
  AddTargetFeature(Args, Features, options::OPT_mdspr2, options::OPT_mno_dspr2,
                   "dspr2");
  AddTargetFeature(Args, Features, options::OPT_mmsa, options::OPT_mno_msa,
                   "msa");

  // Add the last -mfp32/-mfpxx/-mfp64, if none are given and the ABI is O32
  // pass -mfpxx, or if none are given and fp64a is default, pass fp64 and
  // nooddspreg.
  if (Arg *A = Args.getLastArg(options::OPT_mfp32, options::OPT_mfpxx,
                               options::OPT_mfp64)) {
    if (A->getOption().matches(options::OPT_mfp32))
      Features.push_back("-fp64");
    else if (A->getOption().matches(options::OPT_mfpxx)) {
      Features.push_back("+fpxx");
      Features.push_back("+nooddspreg");
    } else
      Features.push_back("+fp64");
  } else if (maxis::shouldUseFPXX(Args, Triple, CPUName, ABIName, FloatABI)) {
    Features.push_back("+fpxx");
    Features.push_back("+nooddspreg");
  } else if (maxis::isFP64ADefault(Triple, CPUName)) {
    Features.push_back("+fp64");
    Features.push_back("+nooddspreg");
  }

  AddTargetFeature(Args, Features, options::OPT_mno_odd_spreg,
                   options::OPT_modd_spreg, "nooddspreg");
  AddTargetFeature(Args, Features, options::OPT_mno_madd4, options::OPT_mmadd4,
                   "nomadd4");
  AddTargetFeature(Args, Features, options::OPT_mmt, options::OPT_mno_mt, "mt");
}

maxis::IEEE754Standard maxis::getIEEE754Standard(StringRef &CPU) {
  // Strictly speaking, maxis32r2 and maxis64r2 do not conform to the
  // IEEE754-2008 standard. Support for this standard was first introduced
  // in Release 3. However, other compilers have traditionally allowed it
  // for Release 2 so we should do the same.
  return (IEEE754Standard)llvm::StringSwitch<int>(CPU)
      .Case("maxis1", Legacy)
      .Case("maxis2", Legacy)
      .Case("maxis3", Legacy)
      .Case("maxis4", Legacy)
      .Case("maxis5", Legacy)
      .Case("maxis32", Legacy)
      .Case("maxis32r2", Legacy | Std2008)
      .Case("maxis32r3", Legacy | Std2008)
      .Case("maxis32r5", Legacy | Std2008)
      .Case("maxis32r6", Std2008)
      .Case("maxis64", Legacy)
      .Case("maxis64r2", Legacy | Std2008)
      .Case("maxis64r3", Legacy | Std2008)
      .Case("maxis64r5", Legacy | Std2008)
      .Case("maxis64r6", Std2008)
      .Default(Std2008);
}

bool maxis::hasCompactBranches(StringRef &CPU) {
  // maxis32r6 and maxis64r6 have compact branches.
  return llvm::StringSwitch<bool>(CPU)
      .Case("maxis32r6", true)
      .Case("maxis64r6", true)
      .Default(false);
}

bool maxis::hasMaxisAbiArg(const ArgList &Args, const char *Value) {
  Arg *A = Args.getLastArg(options::OPT_mabi_EQ);
  return A && (A->getValue() == StringRef(Value));
}

bool maxis::isUCLibc(const ArgList &Args) {
  Arg *A = Args.getLastArg(options::OPT_m_libc_Group);
  return A && A->getOption().matches(options::OPT_muclibc);
}

bool maxis::isNaN2008(const ArgList &Args, const llvm::Triple &Triple) {
  if (Arg *NaNArg = Args.getLastArg(options::OPT_mnan_EQ))
    return llvm::StringSwitch<bool>(NaNArg->getValue())
        .Case("2008", true)
        .Case("legacy", false)
        .Default(false);

  // NaN2008 is the default for MAXIS32r6/MAXIS64r6.
  return llvm::StringSwitch<bool>(getCPUName(Args, Triple))
      .Cases("maxis32r6", "maxis64r6", true)
      .Default(false);

  return false;
}

bool maxis::isFP64ADefault(const llvm::Triple &Triple, StringRef CPUName) {
  if (!Triple.isAndroid())
    return false;

  // Android MAXIS32R6 defaults to FP64A.
  return llvm::StringSwitch<bool>(CPUName)
      .Case("maxis32r6", true)
      .Default(false);
}

bool maxis::isFPXXDefault(const llvm::Triple &Triple, StringRef CPUName,
                         StringRef ABIName, maxis::FloatABI FloatABI) {
  if (Triple.getVendor() != llvm::Triple::ImaginationTechnologies &&
      Triple.getVendor() != llvm::Triple::MaxisTechnologies &&
      !Triple.isAndroid())
    return false;

  if (ABIName != "32")
    return false;

  // FPXX shouldn't be used if either -msoft-float or -mfloat-abi=soft is
  // present.
  if (FloatABI == maxis::FloatABI::Soft)
    return false;

  return llvm::StringSwitch<bool>(CPUName)
      .Cases("maxis2", "maxis3", "maxis4", "maxis5", true)
      .Cases("maxis32", "maxis32r2", "maxis32r3", "maxis32r5", true)
      .Cases("maxis64", "maxis64r2", "maxis64r3", "maxis64r5", true)
      .Default(false);
}

bool maxis::shouldUseFPXX(const ArgList &Args, const llvm::Triple &Triple,
                         StringRef CPUName, StringRef ABIName,
                         maxis::FloatABI FloatABI) {
  bool UseFPXX = isFPXXDefault(Triple, CPUName, ABIName, FloatABI);

  // FPXX shouldn't be used if -msingle-float is present.
  if (Arg *A = Args.getLastArg(options::OPT_msingle_float,
                               options::OPT_mdouble_float))
    if (A->getOption().matches(options::OPT_msingle_float))
      UseFPXX = false;

  return UseFPXX;
}
