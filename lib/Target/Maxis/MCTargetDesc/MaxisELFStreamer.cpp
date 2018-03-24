//===-------- MaxisELFStreamer.cpp - ELF Object Output ---------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MaxisELFStreamer.h"
#include "MaxisOptionRecord.h"
#include "MaxisTargetStreamer.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCSymbolELF.h"
#include "llvm/Support/Casting.h"

using namespace llvm;

MaxisELFStreamer::MaxisELFStreamer(MCContext &Context,
                                 std::unique_ptr<MCAsmBackend> MAB,
                                 raw_pwrite_stream &OS,
                                 std::unique_ptr<MCCodeEmitter> Emitter)
    : MCELFStreamer(Context, std::move(MAB), OS, std::move(Emitter)) {
  RegInfoRecord = new MaxisRegInfoRecord(this, Context);
  MaxisOptionRecords.push_back(
      std::unique_ptr<MaxisRegInfoRecord>(RegInfoRecord));
}

void MaxisELFStreamer::EmitInstruction(const MCInst &Inst,
                                      const MCSubtargetInfo &STI, bool) {
  MCELFStreamer::EmitInstruction(Inst, STI);

  MCContext &Context = getContext();
  const MCRegisterInfo *MCRegInfo = Context.getRegisterInfo();

  for (unsigned OpIndex = 0; OpIndex < Inst.getNumOperands(); ++OpIndex) {
    const MCOperand &Op = Inst.getOperand(OpIndex);

    if (!Op.isReg())
      continue;

    unsigned Reg = Op.getReg();
    RegInfoRecord->SetPhysRegUsed(Reg, MCRegInfo);
  }

  createPendingLabelRelocs();
}

void MaxisELFStreamer::createPendingLabelRelocs() {
  MaxisTargetELFStreamer *ELFTargetStreamer =
      static_cast<MaxisTargetELFStreamer *>(getTargetStreamer());

  // FIXME: Also mark labels when in MAXIS16 mode.
  if (ELFTargetStreamer->isMicroMaxisEnabled()) {
    for (auto *L : Labels) {
      auto *Label = cast<MCSymbolELF>(L);
      getAssembler().registerSymbol(*Label);
      Label->setOther(ELF::STO_MAXIS_MICROMAXIS);
    }
  }

  Labels.clear();
}

void MaxisELFStreamer::EmitLabel(MCSymbol *Symbol, SMLoc Loc) {
  MCELFStreamer::EmitLabel(Symbol);
  Labels.push_back(Symbol);
}

void MaxisELFStreamer::SwitchSection(MCSection *Section,
                                    const MCExpr *Subsection) {
  MCELFStreamer::SwitchSection(Section, Subsection);
  Labels.clear();
}

void MaxisELFStreamer::EmitValueImpl(const MCExpr *Value, unsigned Size,
                                    SMLoc Loc) {
  MCELFStreamer::EmitValueImpl(Value, Size, Loc);
  Labels.clear();
}

void MaxisELFStreamer::EmitMaxisOptionRecords() {
  for (const auto &I : MaxisOptionRecords)
    I->EmitMaxisOptionRecord();
}

MCELFStreamer *llvm::createMaxisELFStreamer(
    MCContext &Context, std::unique_ptr<MCAsmBackend> MAB,
    raw_pwrite_stream &OS, std::unique_ptr<MCCodeEmitter> Emitter,
    bool RelaxAll) {
  return new MaxisELFStreamer(Context, std::move(MAB), OS, std::move(Emitter));
}
