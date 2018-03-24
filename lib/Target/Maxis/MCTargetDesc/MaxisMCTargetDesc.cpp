//===-- MaxisMCTargetDesc.cpp - Maxis Target Descriptions -------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides Maxis specific target descriptions.
//
//===----------------------------------------------------------------------===//

#include "MaxisMCTargetDesc.h"
#include "InstPrinter/MaxisInstPrinter.h"
#include "MaxisAsmBackend.h"
#include "MaxisELFStreamer.h"
#include "MaxisMCAsmInfo.h"
#include "MaxisMCNaCl.h"
#include "MaxisTargetStreamer.h"
#include "llvm/ADT/Triple.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCInstrAnalysis.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MachineLocation.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define GET_INSTRINFO_MC_DESC
#include "MaxisGenInstrInfo.inc"

#define GET_SUBTARGETINFO_MC_DESC
#include "MaxisGenSubtargetInfo.inc"

#define GET_REGINFO_MC_DESC
#include "MaxisGenRegisterInfo.inc"

/// Select the Maxis CPU for the given triple and cpu name.
/// FIXME: Merge with the copy in MaxisSubtarget.cpp
StringRef MAXIS_MC::selectMaxisCPU(const Triple &TT, StringRef CPU) {
  if (CPU.empty() || CPU == "generic") {
    if (TT.getArch() == Triple::maxis || TT.getArch() == Triple::maxisel)
      CPU = "maxis32";
    else
      CPU = "maxis64";
  }
  return CPU;
}

static MCInstrInfo *createMaxisMCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitMaxisMCInstrInfo(X);
  return X;
}

static MCRegisterInfo *createMaxisMCRegisterInfo(const Triple &TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
  InitMaxisMCRegisterInfo(X, Maxis::RA);
  return X;
}

static MCSubtargetInfo *createMaxisMCSubtargetInfo(const Triple &TT,
                                                  StringRef CPU, StringRef FS) {
  CPU = MAXIS_MC::selectMaxisCPU(TT, CPU);
  return createMaxisMCSubtargetInfoImpl(TT, CPU, FS);
}

static MCAsmInfo *createMaxisMCAsmInfo(const MCRegisterInfo &MRI,
                                      const Triple &TT) {
  MCAsmInfo *MAI = new MaxisMCAsmInfo(TT);

  unsigned SP = MRI.getDwarfRegNum(Maxis::SP, true);
  MCCFIInstruction Inst = MCCFIInstruction::createDefCfa(nullptr, SP, 0);
  MAI->addInitialFrameState(Inst);

  return MAI;
}

static MCInstPrinter *createMaxisMCInstPrinter(const Triple &T,
                                              unsigned SyntaxVariant,
                                              const MCAsmInfo &MAI,
                                              const MCInstrInfo &MII,
                                              const MCRegisterInfo &MRI) {
  return new MaxisInstPrinter(MAI, MII, MRI);
}

static MCStreamer *createMCStreamer(const Triple &T, MCContext &Context,
                                    std::unique_ptr<MCAsmBackend> &&MAB,
                                    raw_pwrite_stream &OS,
                                    std::unique_ptr<MCCodeEmitter> &&Emitter,
                                    bool RelaxAll) {
  MCStreamer *S;
  if (!T.isOSNaCl())
    S = createMaxisELFStreamer(Context, std::move(MAB), OS, std::move(Emitter),
                              RelaxAll);
  else
    S = createMaxisNaClELFStreamer(Context, std::move(MAB), OS,
                                  std::move(Emitter), RelaxAll);
  return S;
}

static MCTargetStreamer *createMaxisAsmTargetStreamer(MCStreamer &S,
                                                     formatted_raw_ostream &OS,
                                                     MCInstPrinter *InstPrint,
                                                     bool isVerboseAsm) {
  return new MaxisTargetAsmStreamer(S, OS);
}

static MCTargetStreamer *createMaxisNullTargetStreamer(MCStreamer &S) {
  return new MaxisTargetStreamer(S);
}

static MCTargetStreamer *
createMaxisObjectTargetStreamer(MCStreamer &S, const MCSubtargetInfo &STI) {
  return new MaxisTargetELFStreamer(S, STI);
}

namespace {

class MaxisMCInstrAnalysis : public MCInstrAnalysis {
public:
  MaxisMCInstrAnalysis(const MCInstrInfo *Info) : MCInstrAnalysis(Info) {}

  bool evaluateBranch(const MCInst &Inst, uint64_t Addr, uint64_t Size,
                      uint64_t &Target) const override {
    unsigned NumOps = Inst.getNumOperands();
    if (NumOps == 0)
      return false;
    switch (Info->get(Inst.getOpcode()).OpInfo[NumOps - 1].OperandType) {
    case MCOI::OPERAND_UNKNOWN:
    case MCOI::OPERAND_IMMEDIATE:
      // jal, bal ...
      Target = Inst.getOperand(NumOps - 1).getImm();
      return true;
    case MCOI::OPERAND_PCREL:
      // b, j, beq ...
      Target = Addr + Inst.getOperand(NumOps - 1).getImm();
      return true;
    default:
      return false;
    }
  }
};
}

static MCInstrAnalysis *createMaxisMCInstrAnalysis(const MCInstrInfo *Info) {
  return new MaxisMCInstrAnalysis(Info);
}

extern "C" void LLVMInitializeMaxisTargetMC() {
  for (Target *T : {&getTheMaxisTarget(), &getTheMaxiselTarget(),
                    &getTheMaxis64Target(), &getTheMaxis64elTarget()}) {
    // Register the MC asm info.
    RegisterMCAsmInfoFn X(*T, createMaxisMCAsmInfo);

    // Register the MC instruction info.
    TargetRegistry::RegisterMCInstrInfo(*T, createMaxisMCInstrInfo);

    // Register the MC register info.
    TargetRegistry::RegisterMCRegInfo(*T, createMaxisMCRegisterInfo);

    // Register the elf streamer.
    TargetRegistry::RegisterELFStreamer(*T, createMCStreamer);

    // Register the asm target streamer.
    TargetRegistry::RegisterAsmTargetStreamer(*T, createMaxisAsmTargetStreamer);

    TargetRegistry::RegisterNullTargetStreamer(*T,
                                               createMaxisNullTargetStreamer);

    // Register the MC subtarget info.
    TargetRegistry::RegisterMCSubtargetInfo(*T, createMaxisMCSubtargetInfo);

    // Register the MC instruction analyzer.
    TargetRegistry::RegisterMCInstrAnalysis(*T, createMaxisMCInstrAnalysis);

    // Register the MCInstPrinter.
    TargetRegistry::RegisterMCInstPrinter(*T, createMaxisMCInstPrinter);

    TargetRegistry::RegisterObjectTargetStreamer(
        *T, createMaxisObjectTargetStreamer);

    // Register the asm backend.
    TargetRegistry::RegisterMCAsmBackend(*T, createMaxisAsmBackend);
  }

  // Register the MC Code Emitter
  for (Target *T : {&getTheMaxisTarget(), &getTheMaxis64Target()})
    TargetRegistry::RegisterMCCodeEmitter(*T, createMaxisMCCodeEmitterEB);

  for (Target *T : {&getTheMaxiselTarget(), &getTheMaxis64elTarget()})
    TargetRegistry::RegisterMCCodeEmitter(*T, createMaxisMCCodeEmitterEL);
}
