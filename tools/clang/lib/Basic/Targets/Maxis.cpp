//===--- Maxis.cpp - Implement Maxis target feature support -----------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements Maxis TargetInfo objects.
//
//===----------------------------------------------------------------------===//

#include "Maxis.h"
#include "Targets.h"
#include "clang/Basic/Diagnostic.h"
#include "clang/Basic/MacroBuilder.h"
#include "clang/Basic/TargetBuiltins.h"
#include "llvm/ADT/StringSwitch.h"

using namespace clang;
using namespace clang::targets;

const Builtin::Info MaxisTargetInfo::BuiltinInfo[] = {
#define BUILTIN(ID, TYPE, ATTRS)                                               \
  {#ID, TYPE, ATTRS, nullptr, ALL_LANGUAGES, nullptr},
#define LIBBUILTIN(ID, TYPE, ATTRS, HEADER)                                    \
  {#ID, TYPE, ATTRS, HEADER, ALL_LANGUAGES, nullptr},
#include "clang/Basic/BuiltinsMaxis.def"
};

bool MaxisTargetInfo::processorSupportsGPR64() const {
  return llvm::StringSwitch<bool>(CPU)
      .Case("maxis3", true)
      .Case("maxis4", true)
      .Case("maxis5", true)
      .Case("maxis64", true)
      .Case("maxis64r2", true)
      .Case("maxis64r3", true)
      .Case("maxis64r5", true)
      .Case("maxis64r6", true)
      .Case("octeon", true)
      .Default(false);
  return false;
}

bool MaxisTargetInfo::isValidCPUName(StringRef Name) const {
  return llvm::StringSwitch<bool>(Name)
      .Case("maxis1", true)
      .Case("maxis2", true)
      .Case("maxis3", true)
      .Case("maxis4", true)
      .Case("maxis5", true)
      .Case("maxis32", true)
      .Case("maxis32r2", true)
      .Case("maxis32r3", true)
      .Case("maxis32r5", true)
      .Case("maxis32r6", true)
      .Case("maxis64", true)
      .Case("maxis64r2", true)
      .Case("maxis64r3", true)
      .Case("maxis64r5", true)
      .Case("maxis64r6", true)
      .Case("octeon", true)
      .Case("p5600", true)
      .Default(false);
}

void MaxisTargetInfo::getTargetDefines(const LangOptions &Opts,
                                      MacroBuilder &Builder) const {
  if (BigEndian) {
    DefineStd(Builder, "MAXISEB", Opts);
    Builder.defineMacro("_MAXISEB");
  } else {
    DefineStd(Builder, "MAXISEL", Opts);
    Builder.defineMacro("_MAXISEL");
  }

  Builder.defineMacro("__maxis__");
  Builder.defineMacro("_maxis");
  if (Opts.GNUMode)
    Builder.defineMacro("maxis");

  if (ABI == "o32") {
    Builder.defineMacro("__maxis", "32");
    Builder.defineMacro("_MAXIS_ISA", "_MAXIS_ISA_MAXIS32");
  } else {
    Builder.defineMacro("__maxis", "64");
    Builder.defineMacro("__maxis64");
    Builder.defineMacro("__maxis64__");
    Builder.defineMacro("_MAXIS_ISA", "_MAXIS_ISA_MAXIS64");
  }

  const std::string ISARev = llvm::StringSwitch<std::string>(getCPU())
                                 .Cases("maxis32", "maxis64", "1")
                                 .Cases("maxis32r2", "maxis64r2", "2")
                                 .Cases("maxis32r3", "maxis64r3", "3")
                                 .Cases("maxis32r5", "maxis64r5", "5")
                                 .Cases("maxis32r6", "maxis64r6", "6")
                                 .Default("");
  if (!ISARev.empty())
    Builder.defineMacro("__maxis_isa_rev", ISARev);

  if (ABI == "o32") {
    Builder.defineMacro("__maxis_o32");
    Builder.defineMacro("_ABIO32", "1");
    Builder.defineMacro("_MAXIS_SIM", "_ABIO32");
  } else if (ABI == "n32") {
    Builder.defineMacro("__maxis_n32");
    Builder.defineMacro("_ABIN32", "2");
    Builder.defineMacro("_MAXIS_SIM", "_ABIN32");
  } else if (ABI == "n64") {
    Builder.defineMacro("__maxis_n64");
    Builder.defineMacro("_ABI64", "3");
    Builder.defineMacro("_MAXIS_SIM", "_ABI64");
  } else
    llvm_unreachable("Invalid ABI.");

  if (!IsNoABICalls) {
    Builder.defineMacro("__maxis_abicalls");
    if (CanUseBSDABICalls)
      Builder.defineMacro("__ABICALLS__");
  }

  Builder.defineMacro("__REGISTER_PREFIX__", "");

  switch (FloatABI) {
  case HardFloat:
    Builder.defineMacro("__maxis_hard_float", Twine(1));
    break;
  case SoftFloat:
    Builder.defineMacro("__maxis_soft_float", Twine(1));
    break;
  }

  if (IsSingleFloat)
    Builder.defineMacro("__maxis_single_float", Twine(1));

  Builder.defineMacro("__maxis_fpr", HasFP64 ? Twine(64) : Twine(32));
  Builder.defineMacro("_MAXIS_FPSET",
                      Twine(32 / (HasFP64 || IsSingleFloat ? 1 : 2)));

  if (IsMaxis16)
    Builder.defineMacro("__maxis16", Twine(1));

  if (IsMicromaxis)
    Builder.defineMacro("__maxis_micromaxis", Twine(1));

  if (IsNan2008)
    Builder.defineMacro("__maxis_nan2008", Twine(1));

  if (IsAbs2008)
    Builder.defineMacro("__maxis_abs2008", Twine(1));

  switch (DspRev) {
  default:
    break;
  case DSP1:
    Builder.defineMacro("__maxis_dsp_rev", Twine(1));
    Builder.defineMacro("__maxis_dsp", Twine(1));
    break;
  case DSP2:
    Builder.defineMacro("__maxis_dsp_rev", Twine(2));
    Builder.defineMacro("__maxis_dspr2", Twine(1));
    Builder.defineMacro("__maxis_dsp", Twine(1));
    break;
  }

  if (HasMSA)
    Builder.defineMacro("__maxis_msa", Twine(1));

  if (DisableMadd4)
    Builder.defineMacro("__maxis_no_madd4", Twine(1));

  Builder.defineMacro("_MAXIS_SZPTR", Twine(getPointerWidth(0)));
  Builder.defineMacro("_MAXIS_SZINT", Twine(getIntWidth()));
  Builder.defineMacro("_MAXIS_SZLONG", Twine(getLongWidth()));

  Builder.defineMacro("_MAXIS_ARCH", "\"" + CPU + "\"");
  Builder.defineMacro("_MAXIS_ARCH_" + StringRef(CPU).upper());

  // These shouldn't be defined for MAXIS-I but there's no need to check
  // for that since MAXIS-I isn't supported.
  Builder.defineMacro("__GCC_HAVE_SYNC_COMPARE_AND_SWAP_1");
  Builder.defineMacro("__GCC_HAVE_SYNC_COMPARE_AND_SWAP_2");
  Builder.defineMacro("__GCC_HAVE_SYNC_COMPARE_AND_SWAP_4");

  // 32-bit MAXIS processors don't have the necessary lld/scd instructions
  // found in 64-bit processors. In the case of O32 on a 64-bit processor,
  // the instructions exist but using them violates the ABI since they
  // require 64-bit GPRs and O32 only supports 32-bit GPRs.
  if (ABI == "n32" || ABI == "n64")
    Builder.defineMacro("__GCC_HAVE_SYNC_COMPARE_AND_SWAP_8");
}

bool MaxisTargetInfo::hasFeature(StringRef Feature) const {
  return llvm::StringSwitch<bool>(Feature)
      .Case("maxis", true)
      .Case("fp64", HasFP64)
      .Default(false);
}

ArrayRef<Builtin::Info> MaxisTargetInfo::getTargetBuiltins() const {
  return llvm::makeArrayRef(BuiltinInfo, clang::Maxis::LastTSBuiltin -
                                             Builtin::FirstTSBuiltin);
}

bool MaxisTargetInfo::validateTarget(DiagnosticsEngine &Diags) const {
  // microMAXIS64R6 backend was removed.
  if ((getTriple().getArch() == llvm::Triple::maxis64 ||
       getTriple().getArch() == llvm::Triple::maxis64el) &&
       IsMicromaxis && (ABI == "n32" || ABI == "n64")) {
    Diags.Report(diag::err_target_unsupported_cpu_for_micromaxis) << CPU;
    return false;
  }
  // FIXME: It's valid to use O32 on a 64-bit CPU but the backend can't handle
  //        this yet. It's better to fail here than on the backend assertion.
  if (processorSupportsGPR64() && ABI == "o32") {
    Diags.Report(diag::err_target_unsupported_abi) << ABI << CPU;
    return false;
  }

  // 64-bit ABI's require 64-bit CPU's.
  if (!processorSupportsGPR64() && (ABI == "n32" || ABI == "n64")) {
    Diags.Report(diag::err_target_unsupported_abi) << ABI << CPU;
    return false;
  }

  // FIXME: It's valid to use O32 on a maxis64/maxis64el triple but the backend
  //        can't handle this yet. It's better to fail here than on the
  //        backend assertion.
  if ((getTriple().getArch() == llvm::Triple::maxis64 ||
       getTriple().getArch() == llvm::Triple::maxis64el) &&
      ABI == "o32") {
    Diags.Report(diag::err_target_unsupported_abi_for_triple)
        << ABI << getTriple().str();
    return false;
  }

  // FIXME: It's valid to use N32/N64 on a maxis/maxisel triple but the backend
  //        can't handle this yet. It's better to fail here than on the
  //        backend assertion.
  if ((getTriple().getArch() == llvm::Triple::maxis ||
       getTriple().getArch() == llvm::Triple::maxisel) &&
      (ABI == "n32" || ABI == "n64")) {
    Diags.Report(diag::err_target_unsupported_abi_for_triple)
        << ABI << getTriple().str();
    return false;
  }

  return true;
}
