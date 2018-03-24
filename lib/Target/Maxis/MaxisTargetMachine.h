//===- MaxisTargetMachine.h - Define TargetMachine for Maxis ------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the Maxis specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MAXIS_MAXISTARGETMACHINE_H
#define LLVM_LIB_TARGET_MAXIS_MAXISTARGETMACHINE_H

#include "MCTargetDesc/MaxisABIInfo.h"
#include "MaxisSubtarget.h"
#include "llvm/ADT/Optional.h"
#include "llvm/ADT/StringMap.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Support/CodeGen.h"
#include "llvm/Target/TargetMachine.h"
#include <memory>

namespace llvm {

class MaxisTargetMachine : public LLVMTargetMachine {
  bool isLittle;
  std::unique_ptr<TargetLoweringObjectFile> TLOF;
  // Selected ABI
  MaxisABIInfo ABI;
  MaxisSubtarget *Subtarget;
  MaxisSubtarget DefaultSubtarget;
  MaxisSubtarget NoMaxis16Subtarget;
  MaxisSubtarget Maxis16Subtarget;

  mutable StringMap<std::unique_ptr<MaxisSubtarget>> SubtargetMap;

public:
  MaxisTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                    StringRef FS, const TargetOptions &Options,
                    Optional<Reloc::Model> RM, Optional<CodeModel::Model> CM,
                    CodeGenOpt::Level OL, bool JIT, bool isLittle);
  ~MaxisTargetMachine() override;

  TargetTransformInfo getTargetTransformInfo(const Function &F) override;

  const MaxisSubtarget *getSubtargetImpl() const {
    if (Subtarget)
      return Subtarget;
    return &DefaultSubtarget;
  }

  const MaxisSubtarget *getSubtargetImpl(const Function &F) const override;

  /// \brief Reset the subtarget for the Maxis target.
  void resetSubtarget(MachineFunction *MF);

  // Pass Pipeline Configuration
  TargetPassConfig *createPassConfig(PassManagerBase &PM) override;

  TargetLoweringObjectFile *getObjFileLowering() const override {
    return TLOF.get();
  }

  bool isLittleEndian() const { return isLittle; }
  const MaxisABIInfo &getABI() const { return ABI; }

  bool isMachineVerifierClean() const override {
    return false;
  }
};

/// Maxis32/64 big endian target machine.
///
class MaxisebTargetMachine : public MaxisTargetMachine {
  virtual void anchor();

public:
  MaxisebTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                      StringRef FS, const TargetOptions &Options,
                      Optional<Reloc::Model> RM, Optional<CodeModel::Model> CM,
                      CodeGenOpt::Level OL, bool JIT);
};

/// Maxis32/64 little endian target machine.
///
class MaxiselTargetMachine : public MaxisTargetMachine {
  virtual void anchor();

public:
  MaxiselTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                      StringRef FS, const TargetOptions &Options,
                      Optional<Reloc::Model> RM, Optional<CodeModel::Model> CM,
                      CodeGenOpt::Level OL, bool JIT);
};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_MAXIS_MAXISTARGETMACHINE_H
