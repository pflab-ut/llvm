//===-- MaxisTargetMachine.cpp - Define TargetMachine for Maxis -------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Implements the info about Maxis target spec.
//
//===----------------------------------------------------------------------===//

#include "MaxisTargetMachine.h"
#include "MCTargetDesc/MaxisABIInfo.h"
#include "MCTargetDesc/MaxisMCTargetDesc.h"
#include "Maxis.h"
#include "Maxis16ISelDAGToDAG.h"
#include "MaxisSEISelDAGToDAG.h"
#include "MaxisSubtarget.h"
#include "MaxisTargetObjectFile.h"
#include "llvm/ADT/Optional.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Analysis/TargetTransformInfo.h"
#include "llvm/CodeGen/BasicTTIImpl.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/Attributes.h"
#include "llvm/IR/Function.h"
#include "llvm/Support/CodeGen.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetOptions.h"
#include <string>

using namespace llvm;

#define DEBUG_TYPE "maxis"

extern "C" void LLVMInitializeMaxisTarget() {
  // Register the target.
  RegisterTargetMachine<MaxisebTargetMachine> X(getTheMaxisTarget());
  RegisterTargetMachine<MaxiselTargetMachine> Y(getTheMaxiselTarget());
  RegisterTargetMachine<MaxisebTargetMachine> A(getTheMaxis64Target());
  RegisterTargetMachine<MaxiselTargetMachine> B(getTheMaxis64elTarget());
}

static std::string computeDataLayout(const Triple &TT, StringRef CPU,
                                     const TargetOptions &Options,
                                     bool isLittle) {
  std::string Ret;
  MaxisABIInfo ABI = MaxisABIInfo::computeTargetABI(TT, CPU, Options.MCOptions);

  // There are both little and big endian maxis.
  if (isLittle)
    Ret += "e";
  else
    Ret += "E";

  if (ABI.IsO32())
    Ret += "-m:m";
  else
    Ret += "-m:e";

  // Pointers are 32 bit on some ABIs.
  if (!ABI.IsN64())
    Ret += "-p:32:32";

  // 8 and 16 bit integers only need to have natural alignment, but try to
  // align them to 32 bits. 64 bit integers have natural alignment.
  Ret += "-i8:8:32-i16:16:32-i64:64";

  // 32 bit registers are always available and the stack is at least 64 bit
  // aligned. On N64 64 bit registers are also available and the stack is
  // 128 bit aligned.
  if (ABI.IsN64() || ABI.IsN32())
    Ret += "-n32:64-S128";
  else
    Ret += "-n32-S64";

  return Ret;
}

static Reloc::Model getEffectiveRelocModel(bool JIT,
                                           Optional<Reloc::Model> RM) {
  if (!RM.hasValue() || JIT)
    return Reloc::Static;
  return *RM;
}

static CodeModel::Model getEffectiveCodeModel(Optional<CodeModel::Model> CM) {
  if (CM)
    return *CM;
  return CodeModel::Small;
}

// On function prologue, the stack is created by decrementing
// its pointer. Once decremented, all references are done with positive
// offset from the stack/frame pointer, using StackGrowsUp enables
// an easier handling.
// Using CodeModel::Large enables different CALL behavior.
MaxisTargetMachine::MaxisTargetMachine(const Target &T, const Triple &TT,
                                     StringRef CPU, StringRef FS,
                                     const TargetOptions &Options,
                                     Optional<Reloc::Model> RM,
                                     Optional<CodeModel::Model> CM,
                                     CodeGenOpt::Level OL, bool JIT,
                                     bool isLittle)
    : LLVMTargetMachine(T, computeDataLayout(TT, CPU, Options, isLittle), TT,
                        CPU, FS, Options, getEffectiveRelocModel(JIT, RM),
                        getEffectiveCodeModel(CM), OL),
      isLittle(isLittle), TLOF(llvm::make_unique<MaxisTargetObjectFile>()),
      ABI(MaxisABIInfo::computeTargetABI(TT, CPU, Options.MCOptions)),
      Subtarget(nullptr), DefaultSubtarget(TT, CPU, FS, isLittle, *this,
                                           Options.StackAlignmentOverride),
      NoMaxis16Subtarget(TT, CPU, FS.empty() ? "-maxis16" : FS.str() + ",-maxis16",
                        isLittle, *this, Options.StackAlignmentOverride),
      Maxis16Subtarget(TT, CPU, FS.empty() ? "+maxis16" : FS.str() + ",+maxis16",
                      isLittle, *this, Options.StackAlignmentOverride) {
  Subtarget = &DefaultSubtarget;
  initAsmInfo();
}

MaxisTargetMachine::~MaxisTargetMachine() = default;

void MaxisebTargetMachine::anchor() {}

MaxisebTargetMachine::MaxisebTargetMachine(const Target &T, const Triple &TT,
                                         StringRef CPU, StringRef FS,
                                         const TargetOptions &Options,
                                         Optional<Reloc::Model> RM,
                                         Optional<CodeModel::Model> CM,
                                         CodeGenOpt::Level OL, bool JIT)
    : MaxisTargetMachine(T, TT, CPU, FS, Options, RM, CM, OL, JIT, false) {}

void MaxiselTargetMachine::anchor() {}

MaxiselTargetMachine::MaxiselTargetMachine(const Target &T, const Triple &TT,
                                         StringRef CPU, StringRef FS,
                                         const TargetOptions &Options,
                                         Optional<Reloc::Model> RM,
                                         Optional<CodeModel::Model> CM,
                                         CodeGenOpt::Level OL, bool JIT)
    : MaxisTargetMachine(T, TT, CPU, FS, Options, RM, CM, OL, JIT, true) {}

const MaxisSubtarget *
MaxisTargetMachine::getSubtargetImpl(const Function &F) const {
  Attribute CPUAttr = F.getFnAttribute("target-cpu");
  Attribute FSAttr = F.getFnAttribute("target-features");

  std::string CPU = !CPUAttr.hasAttribute(Attribute::None)
                        ? CPUAttr.getValueAsString().str()
                        : TargetCPU;
  std::string FS = !FSAttr.hasAttribute(Attribute::None)
                       ? FSAttr.getValueAsString().str()
                       : TargetFS;
  bool hasMaxis16Attr =
      !F.getFnAttribute("maxis16").hasAttribute(Attribute::None);
  bool hasNoMaxis16Attr =
      !F.getFnAttribute("nomaxis16").hasAttribute(Attribute::None);

  bool HasMicroMaxisAttr =
      !F.getFnAttribute("micromaxis").hasAttribute(Attribute::None);
  bool HasNoMicroMaxisAttr =
      !F.getFnAttribute("nomicromaxis").hasAttribute(Attribute::None);

  // FIXME: This is related to the code below to reset the target options,
  // we need to know whether or not the soft float flag is set on the
  // function, so we can enable it as a subtarget feature.
  bool softFloat =
      F.hasFnAttribute("use-soft-float") &&
      F.getFnAttribute("use-soft-float").getValueAsString() == "true";

  if (hasMaxis16Attr)
    FS += FS.empty() ? "+maxis16" : ",+maxis16";
  else if (hasNoMaxis16Attr)
    FS += FS.empty() ? "-maxis16" : ",-maxis16";
  if (HasMicroMaxisAttr)
    FS += FS.empty() ? "+micromaxis" : ",+micromaxis";
  else if (HasNoMicroMaxisAttr)
    FS += FS.empty() ? "-micromaxis" : ",-micromaxis";
  if (softFloat)
    FS += FS.empty() ? "+soft-float" : ",+soft-float";

  auto &I = SubtargetMap[CPU + FS];
  if (!I) {
    // This needs to be done before we create a new subtarget since any
    // creation will depend on the TM and the code generation flags on the
    // function that reside in TargetOptions.
    resetTargetOptions(F);
    I = llvm::make_unique<MaxisSubtarget>(TargetTriple, CPU, FS, isLittle, *this,
                                         Options.StackAlignmentOverride);
  }
  return I.get();
}

void MaxisTargetMachine::resetSubtarget(MachineFunction *MF) {
  DEBUG(dbgs() << "resetSubtarget\n");

  Subtarget = const_cast<MaxisSubtarget *>(getSubtargetImpl(MF->getFunction()));
  MF->setSubtarget(Subtarget);
}

namespace {

/// Maxis Code Generator Pass Configuration Options.
class MaxisPassConfig : public TargetPassConfig {
public:
  MaxisPassConfig(MaxisTargetMachine &TM, PassManagerBase &PM)
      : TargetPassConfig(TM, PM) {
    // The current implementation of long branch pass requires a scratch
    // register ($at) to be available before branch instructions. Tail merging
    // can break this requirement, so disable it when long branch pass is
    // enabled.
    EnableTailMerge = !getMaxisSubtarget().enableLongBranchPass();
  }

  MaxisTargetMachine &getMaxisTargetMachine() const {
    return getTM<MaxisTargetMachine>();
  }

  const MaxisSubtarget &getMaxisSubtarget() const {
    return *getMaxisTargetMachine().getSubtargetImpl();
  }

  void addIRPasses() override;
  bool addInstSelector() override;
  void addPreEmitPass() override;
  void addPreRegAlloc() override;
};

} // end anonymous namespace

TargetPassConfig *MaxisTargetMachine::createPassConfig(PassManagerBase &PM) {
  return new MaxisPassConfig(*this, PM);
}

void MaxisPassConfig::addIRPasses() {
  TargetPassConfig::addIRPasses();
  addPass(createAtomicExpandPass());
  if (getMaxisSubtarget().os16())
    addPass(createMaxisOs16Pass());
  if (getMaxisSubtarget().inMaxis16HardFloat())
    addPass(createMaxis16HardFloatPass());
}
// Install an instruction selector pass using
// the ISelDag to gen Maxis code.
bool MaxisPassConfig::addInstSelector() {
  addPass(createMaxisModuleISelDagPass());
  addPass(createMaxis16ISelDag(getMaxisTargetMachine(), getOptLevel()));
  addPass(createMaxisSEISelDag(getMaxisTargetMachine(), getOptLevel()));
  return false;
}

void MaxisPassConfig::addPreRegAlloc() {
  addPass(createMaxisOptimizePICCallPass());
}

TargetTransformInfo
MaxisTargetMachine::getTargetTransformInfo(const Function &F) {
  if (Subtarget->allowMixed16_32()) {
    DEBUG(errs() << "No Target Transform Info Pass Added\n");
    // FIXME: This is no longer necessary as the TTI returned is per-function.
    return TargetTransformInfo(F.getParent()->getDataLayout());
  }

  DEBUG(errs() << "Target Transform Info Pass Added\n");
  return TargetTransformInfo(BasicTTIImpl(this, F));
}

// Implemented by targets that want to run passes immediately before
// machine code is emitted. return true if -print-machineinstrs should
// print out the code after the passes.
void MaxisPassConfig::addPreEmitPass() {
  addPass(createMicroMaxisSizeReductionPass());

  // The delay slot filler and the long branch passes can potientially create
  // forbidden slot/ hazards for MAXISR6 which the hazard schedule pass will
  // fix. Any new pass must come before the hazard schedule pass.
  addPass(createMaxisDelaySlotFillerPass());
  addPass(createMaxisLongBranchPass());
  addPass(createMaxisHazardSchedule());
  addPass(createMaxisConstantIslandPass());
}
