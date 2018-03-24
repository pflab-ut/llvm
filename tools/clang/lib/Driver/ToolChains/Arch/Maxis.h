//===--- Maxis.h - Maxis-specific Tool Helpers ----------------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CLANG_LIB_DRIVER_TOOLCHAINS_ARCH_MAXIS_H
#define LLVM_CLANG_LIB_DRIVER_TOOLCHAINS_ARCH_MAXIS_H

#include "clang/Driver/Driver.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/Triple.h"
#include "llvm/Option/Option.h"
#include <string>
#include <vector>

namespace clang {
namespace driver {
namespace tools {

bool isMaxisArch(llvm::Triple::ArchType Arch);

namespace maxis {
typedef enum { Legacy = 1, Std2008 = 2 } IEEE754Standard;

enum class FloatABI {
  Invalid,
  Soft,
  Hard,
};

IEEE754Standard getIEEE754Standard(StringRef &CPU);
bool hasCompactBranches(StringRef &CPU);
void getMaxisCPUAndABI(const llvm::opt::ArgList &Args,
                      const llvm::Triple &Triple, StringRef &CPUName,
                      StringRef &ABIName);
void getMAXISTargetFeatures(const Driver &D, const llvm::Triple &Triple,
                           const llvm::opt::ArgList &Args,
                           std::vector<StringRef> &Features);
StringRef getGnuCompatibleMaxisABIName(StringRef ABI);
maxis::FloatABI getMaxisFloatABI(const Driver &D, const llvm::opt::ArgList &Args);
std::string getMaxisABILibSuffix(const llvm::opt::ArgList &Args,
                                const llvm::Triple &Triple);
bool hasMaxisAbiArg(const llvm::opt::ArgList &Args, const char *Value);
bool isUCLibc(const llvm::opt::ArgList &Args);
bool isNaN2008(const llvm::opt::ArgList &Args, const llvm::Triple &Triple);
bool isFP64ADefault(const llvm::Triple &Triple, StringRef CPUName);
bool isFPXXDefault(const llvm::Triple &Triple, StringRef CPUName,
                   StringRef ABIName, maxis::FloatABI FloatABI);
bool shouldUseFPXX(const llvm::opt::ArgList &Args, const llvm::Triple &Triple,
                   StringRef CPUName, StringRef ABIName,
                   maxis::FloatABI FloatABI);

} // end namespace maxis
} // end namespace target
} // end namespace driver
} // end namespace clang

#endif // LLVM_CLANG_LIB_DRIVER_TOOLCHAINS_ARCH_MAXIS_H
