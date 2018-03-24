//===-- MaxisTargetStreamer.cpp - Maxis Target Streamer Methods -------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides Maxis specific target streamer methods.
//
//===----------------------------------------------------------------------===//

#include "MaxisTargetStreamer.h"
#include "InstPrinter/MaxisInstPrinter.h"
#include "MCTargetDesc/MaxisABIInfo.h"
#include "MaxisELFStreamer.h"
#include "MaxisMCExpr.h"
#include "MaxisMCTargetDesc.h"
#include "MaxisTargetObjectFile.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbolELF.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FormattedStream.h"

using namespace llvm;

namespace {
static cl::opt<bool> RoundSectionSizes(
    "maxis-round-section-sizes", cl::init(false),
    cl::desc("Round section sizes up to the section alignment"), cl::Hidden);
} // end anonymous namespace

MaxisTargetStreamer::MaxisTargetStreamer(MCStreamer &S)
    : MCTargetStreamer(S), ModuleDirectiveAllowed(true) {
  GPRInfoSet = FPRInfoSet = FrameInfoSet = false;
}
void MaxisTargetStreamer::emitDirectiveSetMicroMaxis() {}
void MaxisTargetStreamer::emitDirectiveSetNoMicroMaxis() {}
void MaxisTargetStreamer::setUsesMicroMaxis() {}
void MaxisTargetStreamer::emitDirectiveSetMaxis16() {}
void MaxisTargetStreamer::emitDirectiveSetNoMaxis16() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetReorder() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetNoReorder() {}
void MaxisTargetStreamer::emitDirectiveSetMacro() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetNoMacro() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetMsa() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetNoMsa() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetMt() {}
void MaxisTargetStreamer::emitDirectiveSetNoMt() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetAt() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetAtWithArg(unsigned RegNo) {
  forbidModuleDirective();
}
void MaxisTargetStreamer::emitDirectiveSetNoAt() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveEnd(StringRef Name) {}
void MaxisTargetStreamer::emitDirectiveEnt(const MCSymbol &Symbol) {}
void MaxisTargetStreamer::emitDirectiveAbiCalls() {}
void MaxisTargetStreamer::emitDirectiveNaN2008() {}
void MaxisTargetStreamer::emitDirectiveNaNLegacy() {}
void MaxisTargetStreamer::emitDirectiveOptionPic0() {}
void MaxisTargetStreamer::emitDirectiveOptionPic2() {}
void MaxisTargetStreamer::emitDirectiveInsn() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitFrame(unsigned StackReg, unsigned StackSize,
                                   unsigned ReturnReg) {}
void MaxisTargetStreamer::emitMask(unsigned CPUBitmask, int CPUTopSavedRegOff) {}
void MaxisTargetStreamer::emitFMask(unsigned FPUBitmask, int FPUTopSavedRegOff) {
}
void MaxisTargetStreamer::emitDirectiveSetArch(StringRef Arch) {
  forbidModuleDirective();
}
void MaxisTargetStreamer::emitDirectiveSetMaxis0() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetMaxis1() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetMaxis2() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetMaxis3() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetMaxis4() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetMaxis5() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetMaxis32() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetMaxis32R2() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetMaxis32R3() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetMaxis32R5() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetMaxis32R6() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetMaxis64() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetMaxis64R2() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetMaxis64R3() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetMaxis64R5() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetMaxis64R6() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetPop() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetPush() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetSoftFloat() {
  forbidModuleDirective();
}
void MaxisTargetStreamer::emitDirectiveSetHardFloat() {
  forbidModuleDirective();
}
void MaxisTargetStreamer::emitDirectiveSetDsp() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetDspr2() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetNoDsp() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveCpLoad(unsigned RegNo) {}
bool MaxisTargetStreamer::emitDirectiveCpRestore(
    int Offset, function_ref<unsigned()> GetATReg, SMLoc IDLoc,
    const MCSubtargetInfo *STI) {
  forbidModuleDirective();
  return true;
}
void MaxisTargetStreamer::emitDirectiveCpsetup(unsigned RegNo, int RegOrOffset,
                                              const MCSymbol &Sym, bool IsReg) {
}
void MaxisTargetStreamer::emitDirectiveCpreturn(unsigned SaveLocation,
                                               bool SaveLocationIsRegister) {}

void MaxisTargetStreamer::emitDirectiveModuleFP() {}

void MaxisTargetStreamer::emitDirectiveModuleOddSPReg() {
  if (!ABIFlagsSection.OddSPReg && !ABIFlagsSection.Is32BitABI)
    report_fatal_error("+nooddspreg is only valid for O32");
}
void MaxisTargetStreamer::emitDirectiveModuleSoftFloat() {}
void MaxisTargetStreamer::emitDirectiveModuleHardFloat() {}
void MaxisTargetStreamer::emitDirectiveModuleMT() {}
void MaxisTargetStreamer::emitDirectiveSetFp(
    MaxisABIFlagsSection::FpABIKind Value) {
  forbidModuleDirective();
}
void MaxisTargetStreamer::emitDirectiveSetOddSPReg() { forbidModuleDirective(); }
void MaxisTargetStreamer::emitDirectiveSetNoOddSPReg() {
  forbidModuleDirective();
}

void MaxisTargetStreamer::emitR(unsigned Opcode, unsigned Reg0, SMLoc IDLoc,
                               const MCSubtargetInfo *STI) {
  MCInst TmpInst;
  TmpInst.setOpcode(Opcode);
  TmpInst.addOperand(MCOperand::createReg(Reg0));
  TmpInst.setLoc(IDLoc);
  getStreamer().EmitInstruction(TmpInst, *STI);
}

void MaxisTargetStreamer::emitRX(unsigned Opcode, unsigned Reg0, MCOperand Op1,
                                SMLoc IDLoc, const MCSubtargetInfo *STI) {
  MCInst TmpInst;
  TmpInst.setOpcode(Opcode);
  TmpInst.addOperand(MCOperand::createReg(Reg0));
  TmpInst.addOperand(Op1);
  TmpInst.setLoc(IDLoc);
  getStreamer().EmitInstruction(TmpInst, *STI);
}

void MaxisTargetStreamer::emitRI(unsigned Opcode, unsigned Reg0, int32_t Imm,
                                SMLoc IDLoc, const MCSubtargetInfo *STI) {
  emitRX(Opcode, Reg0, MCOperand::createImm(Imm), IDLoc, STI);
}

void MaxisTargetStreamer::emitRR(unsigned Opcode, unsigned Reg0, unsigned Reg1,
                                SMLoc IDLoc, const MCSubtargetInfo *STI) {
  emitRX(Opcode, Reg0, MCOperand::createReg(Reg1), IDLoc, STI);
}

void MaxisTargetStreamer::emitII(unsigned Opcode, int16_t Imm1, int16_t Imm2,
                                SMLoc IDLoc, const MCSubtargetInfo *STI) {
  MCInst TmpInst;
  TmpInst.setOpcode(Opcode);
  TmpInst.addOperand(MCOperand::createImm(Imm1));
  TmpInst.addOperand(MCOperand::createImm(Imm2));
  TmpInst.setLoc(IDLoc);
  getStreamer().EmitInstruction(TmpInst, *STI);
}

void MaxisTargetStreamer::emitRRX(unsigned Opcode, unsigned Reg0, unsigned Reg1,
                                 MCOperand Op2, SMLoc IDLoc,
                                 const MCSubtargetInfo *STI) {
  MCInst TmpInst;
  TmpInst.setOpcode(Opcode);
  TmpInst.addOperand(MCOperand::createReg(Reg0));
  TmpInst.addOperand(MCOperand::createReg(Reg1));
  TmpInst.addOperand(Op2);
  TmpInst.setLoc(IDLoc);
  getStreamer().EmitInstruction(TmpInst, *STI);
}

void MaxisTargetStreamer::emitRRR(unsigned Opcode, unsigned Reg0, unsigned Reg1,
                                 unsigned Reg2, SMLoc IDLoc,
                                 const MCSubtargetInfo *STI) {
  emitRRX(Opcode, Reg0, Reg1, MCOperand::createReg(Reg2), IDLoc, STI);
}

void MaxisTargetStreamer::emitRRI(unsigned Opcode, unsigned Reg0, unsigned Reg1,
                                 int16_t Imm, SMLoc IDLoc,
                                 const MCSubtargetInfo *STI) {
  emitRRX(Opcode, Reg0, Reg1, MCOperand::createImm(Imm), IDLoc, STI);
}

void MaxisTargetStreamer::emitRRIII(unsigned Opcode, unsigned Reg0,
                                   unsigned Reg1, int16_t Imm0, int16_t Imm1,
                                   int16_t Imm2, SMLoc IDLoc,
                                   const MCSubtargetInfo *STI) {
  MCInst TmpInst;
  TmpInst.setOpcode(Opcode);
  TmpInst.addOperand(MCOperand::createReg(Reg0));
  TmpInst.addOperand(MCOperand::createReg(Reg1));
  TmpInst.addOperand(MCOperand::createImm(Imm0));
  TmpInst.addOperand(MCOperand::createImm(Imm1));
  TmpInst.addOperand(MCOperand::createImm(Imm2));
  TmpInst.setLoc(IDLoc);
  getStreamer().EmitInstruction(TmpInst, *STI);
}

void MaxisTargetStreamer::emitAddu(unsigned DstReg, unsigned SrcReg,
                                  unsigned TrgReg, bool Is64Bit,
                                  const MCSubtargetInfo *STI) {
  emitRRR(Is64Bit ? Maxis::DADDu : Maxis::ADDu, DstReg, SrcReg, TrgReg, SMLoc(),
          STI);
}

void MaxisTargetStreamer::emitDSLL(unsigned DstReg, unsigned SrcReg,
                                  int16_t ShiftAmount, SMLoc IDLoc,
                                  const MCSubtargetInfo *STI) {
  if (ShiftAmount >= 32) {
    emitRRI(Maxis::DSLL32, DstReg, SrcReg, ShiftAmount - 32, IDLoc, STI);
    return;
  }

  emitRRI(Maxis::DSLL, DstReg, SrcReg, ShiftAmount, IDLoc, STI);
}

void MaxisTargetStreamer::emitEmptyDelaySlot(bool hasShortDelaySlot, SMLoc IDLoc,
                                            const MCSubtargetInfo *STI) {
  if (hasShortDelaySlot)
    emitRR(Maxis::MOVE16_MM, Maxis::ZERO, Maxis::ZERO, IDLoc, STI);
  else
    emitRRI(Maxis::SLL, Maxis::ZERO, Maxis::ZERO, 0, IDLoc, STI);
}

void MaxisTargetStreamer::emitNop(SMLoc IDLoc, const MCSubtargetInfo *STI) {
  emitRRI(Maxis::SLL, Maxis::ZERO, Maxis::ZERO, 0, IDLoc, STI);
}

/// Emit the $gp restore operation for .cprestore.
void MaxisTargetStreamer::emitGPRestore(int Offset, SMLoc IDLoc,
                                       const MCSubtargetInfo *STI) {
  emitLoadWithImmOffset(Maxis::LW, Maxis::GP, Maxis::SP, Offset, Maxis::GP, IDLoc,
                        STI);
}

/// Emit a store instruction with an immediate offset.
void MaxisTargetStreamer::emitStoreWithImmOffset(
    unsigned Opcode, unsigned SrcReg, unsigned BaseReg, int64_t Offset,
    function_ref<unsigned()> GetATReg, SMLoc IDLoc,
    const MCSubtargetInfo *STI) {
  if (isInt<16>(Offset)) {
    emitRRI(Opcode, SrcReg, BaseReg, Offset, IDLoc, STI);
    return;
  }

  // sw $8, offset($8) => lui $at, %hi(offset)
  //                      add $at, $at, $8
  //                      sw $8, %lo(offset)($at)

  unsigned ATReg = GetATReg();
  if (!ATReg)
    return;

  unsigned LoOffset = Offset & 0x0000ffff;
  unsigned HiOffset = (Offset & 0xffff0000) >> 16;

  // If msb of LoOffset is 1(negative number) we must increment HiOffset
  // to account for the sign-extension of the low part.
  if (LoOffset & 0x8000)
    HiOffset++;

  // Generate the base address in ATReg.
  emitRI(Maxis::LUi, ATReg, HiOffset, IDLoc, STI);
  if (BaseReg != Maxis::ZERO)
    emitRRR(Maxis::ADDu, ATReg, ATReg, BaseReg, IDLoc, STI);
  // Emit the store with the adjusted base and offset.
  emitRRI(Opcode, SrcReg, ATReg, LoOffset, IDLoc, STI);
}

/// Emit a store instruction with an symbol offset. Symbols are assumed to be
/// out of range for a simm16 will be expanded to appropriate instructions.
void MaxisTargetStreamer::emitStoreWithSymOffset(
    unsigned Opcode, unsigned SrcReg, unsigned BaseReg, MCOperand &HiOperand,
    MCOperand &LoOperand, unsigned ATReg, SMLoc IDLoc,
    const MCSubtargetInfo *STI) {
  // sw $8, sym => lui $at, %hi(sym)
  //               sw $8, %lo(sym)($at)

  // Generate the base address in ATReg.
  emitRX(Maxis::LUi, ATReg, HiOperand, IDLoc, STI);
  if (BaseReg != Maxis::ZERO)
    emitRRR(Maxis::ADDu, ATReg, ATReg, BaseReg, IDLoc, STI);
  // Emit the store with the adjusted base and offset.
  emitRRX(Opcode, SrcReg, ATReg, LoOperand, IDLoc, STI);
}

/// Emit a load instruction with an immediate offset. DstReg and TmpReg are
/// permitted to be the same register iff DstReg is distinct from BaseReg and
/// DstReg is a GPR. It is the callers responsibility to identify such cases
/// and pass the appropriate register in TmpReg.
void MaxisTargetStreamer::emitLoadWithImmOffset(unsigned Opcode, unsigned DstReg,
                                               unsigned BaseReg, int64_t Offset,
                                               unsigned TmpReg, SMLoc IDLoc,
                                               const MCSubtargetInfo *STI) {
  if (isInt<16>(Offset)) {
    emitRRI(Opcode, DstReg, BaseReg, Offset, IDLoc, STI);
    return;
  }

  // 1) lw $8, offset($9) => lui $8, %hi(offset)
  //                         add $8, $8, $9
  //                         lw $8, %lo(offset)($9)
  // 2) lw $8, offset($8) => lui $at, %hi(offset)
  //                         add $at, $at, $8
  //                         lw $8, %lo(offset)($at)

  unsigned LoOffset = Offset & 0x0000ffff;
  unsigned HiOffset = (Offset & 0xffff0000) >> 16;

  // If msb of LoOffset is 1(negative number) we must increment HiOffset
  // to account for the sign-extension of the low part.
  if (LoOffset & 0x8000)
    HiOffset++;

  // Generate the base address in TmpReg.
  emitRI(Maxis::LUi, TmpReg, HiOffset, IDLoc, STI);
  if (BaseReg != Maxis::ZERO)
    emitRRR(Maxis::ADDu, TmpReg, TmpReg, BaseReg, IDLoc, STI);
  // Emit the load with the adjusted base and offset.
  emitRRI(Opcode, DstReg, TmpReg, LoOffset, IDLoc, STI);
}

/// Emit a load instruction with an symbol offset. Symbols are assumed to be
/// out of range for a simm16 will be expanded to appropriate instructions.
/// DstReg and TmpReg are permitted to be the same register iff DstReg is a
/// GPR. It is the callers responsibility to identify such cases and pass the
/// appropriate register in TmpReg.
void MaxisTargetStreamer::emitLoadWithSymOffset(unsigned Opcode, unsigned DstReg,
                                               unsigned BaseReg,
                                               MCOperand &HiOperand,
                                               MCOperand &LoOperand,
                                               unsigned TmpReg, SMLoc IDLoc,
                                               const MCSubtargetInfo *STI) {
  // 1) lw $8, sym        => lui $8, %hi(sym)
  //                         lw $8, %lo(sym)($8)
  // 2) ldc1 $f0, sym     => lui $at, %hi(sym)
  //                         ldc1 $f0, %lo(sym)($at)

  // Generate the base address in TmpReg.
  emitRX(Maxis::LUi, TmpReg, HiOperand, IDLoc, STI);
  if (BaseReg != Maxis::ZERO)
    emitRRR(Maxis::ADDu, TmpReg, TmpReg, BaseReg, IDLoc, STI);
  // Emit the load with the adjusted base and offset.
  emitRRX(Opcode, DstReg, TmpReg, LoOperand, IDLoc, STI);
}

MaxisTargetAsmStreamer::MaxisTargetAsmStreamer(MCStreamer &S,
                                             formatted_raw_ostream &OS)
    : MaxisTargetStreamer(S), OS(OS) {}

void MaxisTargetAsmStreamer::emitDirectiveSetMicroMaxis() {
  OS << "\t.set\tmicromaxis\n";
  forbidModuleDirective();
}

void MaxisTargetAsmStreamer::emitDirectiveSetNoMicroMaxis() {
  OS << "\t.set\tnomicromaxis\n";
  forbidModuleDirective();
}

void MaxisTargetAsmStreamer::emitDirectiveSetMaxis16() {
  OS << "\t.set\tmaxis16\n";
  forbidModuleDirective();
}

void MaxisTargetAsmStreamer::emitDirectiveSetNoMaxis16() {
  OS << "\t.set\tnomaxis16\n";
  MaxisTargetStreamer::emitDirectiveSetNoMaxis16();
}

void MaxisTargetAsmStreamer::emitDirectiveSetReorder() {
  OS << "\t.set\treorder\n";
  MaxisTargetStreamer::emitDirectiveSetReorder();
}

void MaxisTargetAsmStreamer::emitDirectiveSetNoReorder() {
  OS << "\t.set\tnoreorder\n";
  forbidModuleDirective();
}

void MaxisTargetAsmStreamer::emitDirectiveSetMacro() {
  OS << "\t.set\tmacro\n";
  MaxisTargetStreamer::emitDirectiveSetMacro();
}

void MaxisTargetAsmStreamer::emitDirectiveSetNoMacro() {
  OS << "\t.set\tnomacro\n";
  MaxisTargetStreamer::emitDirectiveSetNoMacro();
}

void MaxisTargetAsmStreamer::emitDirectiveSetMsa() {
  OS << "\t.set\tmsa\n";
  MaxisTargetStreamer::emitDirectiveSetMsa();
}

void MaxisTargetAsmStreamer::emitDirectiveSetNoMsa() {
  OS << "\t.set\tnomsa\n";
  MaxisTargetStreamer::emitDirectiveSetNoMsa();
}

void MaxisTargetAsmStreamer::emitDirectiveSetMt() {
  OS << "\t.set\tmt\n";
  MaxisTargetStreamer::emitDirectiveSetMt();
}

void MaxisTargetAsmStreamer::emitDirectiveSetNoMt() {
  OS << "\t.set\tnomt\n";
  MaxisTargetStreamer::emitDirectiveSetNoMt();
}

void MaxisTargetAsmStreamer::emitDirectiveSetAt() {
  OS << "\t.set\tat\n";
  MaxisTargetStreamer::emitDirectiveSetAt();
}

void MaxisTargetAsmStreamer::emitDirectiveSetAtWithArg(unsigned RegNo) {
  OS << "\t.set\tat=$" << Twine(RegNo) << "\n";
  MaxisTargetStreamer::emitDirectiveSetAtWithArg(RegNo);
}

void MaxisTargetAsmStreamer::emitDirectiveSetNoAt() {
  OS << "\t.set\tnoat\n";
  MaxisTargetStreamer::emitDirectiveSetNoAt();
}

void MaxisTargetAsmStreamer::emitDirectiveEnd(StringRef Name) {
  OS << "\t.end\t" << Name << '\n';
}

void MaxisTargetAsmStreamer::emitDirectiveEnt(const MCSymbol &Symbol) {
  OS << "\t.ent\t" << Symbol.getName() << '\n';
}

void MaxisTargetAsmStreamer::emitDirectiveAbiCalls() { OS << "\t.abicalls\n"; }

void MaxisTargetAsmStreamer::emitDirectiveNaN2008() { OS << "\t.nan\t2008\n"; }

void MaxisTargetAsmStreamer::emitDirectiveNaNLegacy() {
  OS << "\t.nan\tlegacy\n";
}

void MaxisTargetAsmStreamer::emitDirectiveOptionPic0() {
  OS << "\t.option\tpic0\n";
}

void MaxisTargetAsmStreamer::emitDirectiveOptionPic2() {
  OS << "\t.option\tpic2\n";
}

void MaxisTargetAsmStreamer::emitDirectiveInsn() {
  MaxisTargetStreamer::emitDirectiveInsn();
  OS << "\t.insn\n";
}

void MaxisTargetAsmStreamer::emitFrame(unsigned StackReg, unsigned StackSize,
                                      unsigned ReturnReg) {
  OS << "\t.frame\t$"
     << StringRef(MaxisInstPrinter::getRegisterName(StackReg)).lower() << ","
     << StackSize << ",$"
     << StringRef(MaxisInstPrinter::getRegisterName(ReturnReg)).lower() << '\n';
}

void MaxisTargetAsmStreamer::emitDirectiveSetArch(StringRef Arch) {
  OS << "\t.set arch=" << Arch << "\n";
  MaxisTargetStreamer::emitDirectiveSetArch(Arch);
}

void MaxisTargetAsmStreamer::emitDirectiveSetMaxis0() {
  OS << "\t.set\tmaxis0\n";
  MaxisTargetStreamer::emitDirectiveSetMaxis0();
}

void MaxisTargetAsmStreamer::emitDirectiveSetMaxis1() {
  OS << "\t.set\tmaxis1\n";
  MaxisTargetStreamer::emitDirectiveSetMaxis1();
}

void MaxisTargetAsmStreamer::emitDirectiveSetMaxis2() {
  OS << "\t.set\tmaxis2\n";
  MaxisTargetStreamer::emitDirectiveSetMaxis2();
}

void MaxisTargetAsmStreamer::emitDirectiveSetMaxis3() {
  OS << "\t.set\tmaxis3\n";
  MaxisTargetStreamer::emitDirectiveSetMaxis3();
}

void MaxisTargetAsmStreamer::emitDirectiveSetMaxis4() {
  OS << "\t.set\tmaxis4\n";
  MaxisTargetStreamer::emitDirectiveSetMaxis4();
}

void MaxisTargetAsmStreamer::emitDirectiveSetMaxis5() {
  OS << "\t.set\tmaxis5\n";
  MaxisTargetStreamer::emitDirectiveSetMaxis5();
}

void MaxisTargetAsmStreamer::emitDirectiveSetMaxis32() {
  OS << "\t.set\tmaxis32\n";
  MaxisTargetStreamer::emitDirectiveSetMaxis32();
}

void MaxisTargetAsmStreamer::emitDirectiveSetMaxis32R2() {
  OS << "\t.set\tmaxis32r2\n";
  MaxisTargetStreamer::emitDirectiveSetMaxis32R2();
}

void MaxisTargetAsmStreamer::emitDirectiveSetMaxis32R3() {
  OS << "\t.set\tmaxis32r3\n";
  MaxisTargetStreamer::emitDirectiveSetMaxis32R3();
}

void MaxisTargetAsmStreamer::emitDirectiveSetMaxis32R5() {
  OS << "\t.set\tmaxis32r5\n";
  MaxisTargetStreamer::emitDirectiveSetMaxis32R5();
}

void MaxisTargetAsmStreamer::emitDirectiveSetMaxis32R6() {
  OS << "\t.set\tmaxis32r6\n";
  MaxisTargetStreamer::emitDirectiveSetMaxis32R6();
}

void MaxisTargetAsmStreamer::emitDirectiveSetMaxis64() {
  OS << "\t.set\tmaxis64\n";
  MaxisTargetStreamer::emitDirectiveSetMaxis64();
}

void MaxisTargetAsmStreamer::emitDirectiveSetMaxis64R2() {
  OS << "\t.set\tmaxis64r2\n";
  MaxisTargetStreamer::emitDirectiveSetMaxis64R2();
}

void MaxisTargetAsmStreamer::emitDirectiveSetMaxis64R3() {
  OS << "\t.set\tmaxis64r3\n";
  MaxisTargetStreamer::emitDirectiveSetMaxis64R3();
}

void MaxisTargetAsmStreamer::emitDirectiveSetMaxis64R5() {
  OS << "\t.set\tmaxis64r5\n";
  MaxisTargetStreamer::emitDirectiveSetMaxis64R5();
}

void MaxisTargetAsmStreamer::emitDirectiveSetMaxis64R6() {
  OS << "\t.set\tmaxis64r6\n";
  MaxisTargetStreamer::emitDirectiveSetMaxis64R6();
}

void MaxisTargetAsmStreamer::emitDirectiveSetDsp() {
  OS << "\t.set\tdsp\n";
  MaxisTargetStreamer::emitDirectiveSetDsp();
}

void MaxisTargetAsmStreamer::emitDirectiveSetDspr2() {
  OS << "\t.set\tdspr2\n";
  MaxisTargetStreamer::emitDirectiveSetDspr2();
}

void MaxisTargetAsmStreamer::emitDirectiveSetNoDsp() {
  OS << "\t.set\tnodsp\n";
  MaxisTargetStreamer::emitDirectiveSetNoDsp();
}

void MaxisTargetAsmStreamer::emitDirectiveSetPop() {
  OS << "\t.set\tpop\n";
  MaxisTargetStreamer::emitDirectiveSetPop();
}

void MaxisTargetAsmStreamer::emitDirectiveSetPush() {
 OS << "\t.set\tpush\n";
 MaxisTargetStreamer::emitDirectiveSetPush();
}

void MaxisTargetAsmStreamer::emitDirectiveSetSoftFloat() {
  OS << "\t.set\tsoftfloat\n";
  MaxisTargetStreamer::emitDirectiveSetSoftFloat();
}

void MaxisTargetAsmStreamer::emitDirectiveSetHardFloat() {
  OS << "\t.set\thardfloat\n";
  MaxisTargetStreamer::emitDirectiveSetHardFloat();
}

// Print a 32 bit hex number with all numbers.
static void printHex32(unsigned Value, raw_ostream &OS) {
  OS << "0x";
  for (int i = 7; i >= 0; i--)
    OS.write_hex((Value & (0xF << (i * 4))) >> (i * 4));
}

void MaxisTargetAsmStreamer::emitMask(unsigned CPUBitmask,
                                     int CPUTopSavedRegOff) {
  OS << "\t.mask \t";
  printHex32(CPUBitmask, OS);
  OS << ',' << CPUTopSavedRegOff << '\n';
}

void MaxisTargetAsmStreamer::emitFMask(unsigned FPUBitmask,
                                      int FPUTopSavedRegOff) {
  OS << "\t.fmask\t";
  printHex32(FPUBitmask, OS);
  OS << "," << FPUTopSavedRegOff << '\n';
}

void MaxisTargetAsmStreamer::emitDirectiveCpLoad(unsigned RegNo) {
  OS << "\t.cpload\t$"
     << StringRef(MaxisInstPrinter::getRegisterName(RegNo)).lower() << "\n";
  forbidModuleDirective();
}

bool MaxisTargetAsmStreamer::emitDirectiveCpRestore(
    int Offset, function_ref<unsigned()> GetATReg, SMLoc IDLoc,
    const MCSubtargetInfo *STI) {
  MaxisTargetStreamer::emitDirectiveCpRestore(Offset, GetATReg, IDLoc, STI);
  OS << "\t.cprestore\t" << Offset << "\n";
  return true;
}

void MaxisTargetAsmStreamer::emitDirectiveCpsetup(unsigned RegNo,
                                                 int RegOrOffset,
                                                 const MCSymbol &Sym,
                                                 bool IsReg) {
  OS << "\t.cpsetup\t$"
     << StringRef(MaxisInstPrinter::getRegisterName(RegNo)).lower() << ", ";

  if (IsReg)
    OS << "$"
       << StringRef(MaxisInstPrinter::getRegisterName(RegOrOffset)).lower();
  else
    OS << RegOrOffset;

  OS << ", ";

  OS << Sym.getName();
  forbidModuleDirective();
}

void MaxisTargetAsmStreamer::emitDirectiveCpreturn(unsigned SaveLocation,
                                                  bool SaveLocationIsRegister) {
  OS << "\t.cpreturn";
  forbidModuleDirective();
}

void MaxisTargetAsmStreamer::emitDirectiveModuleFP() {
  OS << "\t.module\tfp=";
  OS << ABIFlagsSection.getFpABIString(ABIFlagsSection.getFpABI()) << "\n";
}

void MaxisTargetAsmStreamer::emitDirectiveSetFp(
    MaxisABIFlagsSection::FpABIKind Value) {
  MaxisTargetStreamer::emitDirectiveSetFp(Value);

  OS << "\t.set\tfp=";
  OS << ABIFlagsSection.getFpABIString(Value) << "\n";
}

void MaxisTargetAsmStreamer::emitDirectiveModuleOddSPReg() {
  MaxisTargetStreamer::emitDirectiveModuleOddSPReg();

  OS << "\t.module\t" << (ABIFlagsSection.OddSPReg ? "" : "no") << "oddspreg\n";
}

void MaxisTargetAsmStreamer::emitDirectiveSetOddSPReg() {
  MaxisTargetStreamer::emitDirectiveSetOddSPReg();
  OS << "\t.set\toddspreg\n";
}

void MaxisTargetAsmStreamer::emitDirectiveSetNoOddSPReg() {
  MaxisTargetStreamer::emitDirectiveSetNoOddSPReg();
  OS << "\t.set\tnooddspreg\n";
}

void MaxisTargetAsmStreamer::emitDirectiveModuleSoftFloat() {
  OS << "\t.module\tsoftfloat\n";
}

void MaxisTargetAsmStreamer::emitDirectiveModuleHardFloat() {
  OS << "\t.module\thardfloat\n";
}

void MaxisTargetAsmStreamer::emitDirectiveModuleMT() {
  OS << "\t.module\tmt\n";
}

// This part is for ELF object output.
MaxisTargetELFStreamer::MaxisTargetELFStreamer(MCStreamer &S,
                                             const MCSubtargetInfo &STI)
    : MaxisTargetStreamer(S), MicroMaxisEnabled(false), STI(STI) {
  MCAssembler &MCA = getStreamer().getAssembler();

  // It's possible that MCObjectFileInfo isn't fully initialized at this point
  // due to an initialization order problem where LLVMTargetMachine creates the
  // target streamer before TargetLoweringObjectFile calls
  // InitializeMCObjectFileInfo. There doesn't seem to be a single place that
  // covers all cases so this statement covers most cases and direct object
  // emission must call setPic() once MCObjectFileInfo has been initialized. The
  // cases we don't handle here are covered by MaxisAsmPrinter.
  Pic = MCA.getContext().getObjectFileInfo()->isPositionIndependent();

  const FeatureBitset &Features = STI.getFeatureBits();

  // Set the header flags that we can in the constructor.
  // FIXME: This is a fairly terrible hack. We set the rest
  // of these in the destructor. The problem here is two-fold:
  //
  // a: Some of the eflags can be set/reset by directives.
  // b: There aren't any usage paths that initialize the ABI
  //    pointer until after we initialize either an assembler
  //    or the target machine.
  // We can fix this by making the target streamer construct
  // the ABI, but this is fraught with wide ranging dependency
  // issues as well.
  unsigned EFlags = MCA.getELFHeaderEFlags();

  // FIXME: Fix a dependency issue by instantiating the ABI object to some
  // default based off the triple. The triple doesn't describe the target
  // fully, but any external user of the API that uses the MCTargetStreamer
  // would otherwise crash on assertion failure.

  ABI = MaxisABIInfo(
      STI.getTargetTriple().getArch() == Triple::ArchType::maxisel ||
              STI.getTargetTriple().getArch() == Triple::ArchType::maxis
          ? MaxisABIInfo::O32()
          : MaxisABIInfo::N64());

  // Architecture
  if (Features[Maxis::FeatureMaxis64r6])
    EFlags |= ELF::EF_MAXIS_ARCH_64R6;
  else if (Features[Maxis::FeatureMaxis64r2] ||
           Features[Maxis::FeatureMaxis64r3] ||
           Features[Maxis::FeatureMaxis64r5])
    EFlags |= ELF::EF_MAXIS_ARCH_64R2;
  else if (Features[Maxis::FeatureMaxis64])
    EFlags |= ELF::EF_MAXIS_ARCH_64;
  else if (Features[Maxis::FeatureMaxis5])
    EFlags |= ELF::EF_MAXIS_ARCH_5;
  else if (Features[Maxis::FeatureMaxis4])
    EFlags |= ELF::EF_MAXIS_ARCH_4;
  else if (Features[Maxis::FeatureMaxis3])
    EFlags |= ELF::EF_MAXIS_ARCH_3;
  else if (Features[Maxis::FeatureMaxis32r6])
    EFlags |= ELF::EF_MAXIS_ARCH_32R6;
  else if (Features[Maxis::FeatureMaxis32r2] ||
           Features[Maxis::FeatureMaxis32r3] ||
           Features[Maxis::FeatureMaxis32r5])
    EFlags |= ELF::EF_MAXIS_ARCH_32R2;
  else if (Features[Maxis::FeatureMaxis32])
    EFlags |= ELF::EF_MAXIS_ARCH_32;
  else if (Features[Maxis::FeatureMaxis2])
    EFlags |= ELF::EF_MAXIS_ARCH_2;
  else
    EFlags |= ELF::EF_MAXIS_ARCH_1;

  // Machine
  if (Features[Maxis::FeatureCnMaxis])
    EFlags |= ELF::EF_MAXIS_MACH_OCTEON;

  // Other options.
  if (Features[Maxis::FeatureNaN2008])
    EFlags |= ELF::EF_MAXIS_NAN2008;

  MCA.setELFHeaderEFlags(EFlags);
}

void MaxisTargetELFStreamer::emitLabel(MCSymbol *S) {
  auto *Symbol = cast<MCSymbolELF>(S);
  getStreamer().getAssembler().registerSymbol(*Symbol);
  uint8_t Type = Symbol->getType();
  if (Type != ELF::STT_FUNC)
    return;

  if (isMicroMaxisEnabled())
    Symbol->setOther(ELF::STO_MAXIS_MICROMAXIS);
}

void MaxisTargetELFStreamer::finish() {
  MCAssembler &MCA = getStreamer().getAssembler();
  const MCObjectFileInfo &OFI = *MCA.getContext().getObjectFileInfo();

  // .bss, .text and .data are always at least 16-byte aligned.
  MCSection &TextSection = *OFI.getTextSection();
  MCA.registerSection(TextSection);
  MCSection &DataSection = *OFI.getDataSection();
  MCA.registerSection(DataSection);
  MCSection &BSSSection = *OFI.getBSSSection();
  MCA.registerSection(BSSSection);

  TextSection.setAlignment(std::max(16u, TextSection.getAlignment()));
  DataSection.setAlignment(std::max(16u, DataSection.getAlignment()));
  BSSSection.setAlignment(std::max(16u, BSSSection.getAlignment()));

  if (RoundSectionSizes) {
    // Make sections sizes a multiple of the alignment. This is useful for
    // verifying the output of IAS against the output of other assemblers but
    // it's not necessary to produce a correct object and increases section
    // size.
    MCStreamer &OS = getStreamer();
    for (MCSection &S : MCA) {
      MCSectionELF &Section = static_cast<MCSectionELF &>(S);

      unsigned Alignment = Section.getAlignment();
      if (Alignment) {
        OS.SwitchSection(&Section);
        if (Section.UseCodeAlign())
          OS.EmitCodeAlignment(Alignment, Alignment);
        else
          OS.EmitValueToAlignment(Alignment, 0, 1, Alignment);
      }
    }
  }

  const FeatureBitset &Features = STI.getFeatureBits();

  // Update e_header flags. See the FIXME and comment above in
  // the constructor for a full rundown on this.
  unsigned EFlags = MCA.getELFHeaderEFlags();

  // ABI
  // N64 does not require any ABI bits.
  if (getABI().IsO32())
    EFlags |= ELF::EF_MAXIS_ABI_O32;
  else if (getABI().IsN32())
    EFlags |= ELF::EF_MAXIS_ABI2;

  if (Features[Maxis::FeatureGP64Bit]) {
    if (getABI().IsO32())
      EFlags |= ELF::EF_MAXIS_32BITMODE; /* Compatibility Mode */
  } else if (Features[Maxis::FeatureMaxis64r2] || Features[Maxis::FeatureMaxis64])
    EFlags |= ELF::EF_MAXIS_32BITMODE;

  // -mplt is not implemented but we should act as if it was
  // given.
  if (!Features[Maxis::FeatureNoABICalls])
    EFlags |= ELF::EF_MAXIS_CPIC;

  if (Pic)
    EFlags |= ELF::EF_MAXIS_PIC | ELF::EF_MAXIS_CPIC;

  MCA.setELFHeaderEFlags(EFlags);

  // Emit all the option records.
  // At the moment we are only emitting .Maxis.options (ODK_REGINFO) and
  // .reginfo.
  MaxisELFStreamer &MEF = static_cast<MaxisELFStreamer &>(Streamer);
  MEF.EmitMaxisOptionRecords();

  emitMaxisAbiFlags();
}

void MaxisTargetELFStreamer::emitAssignment(MCSymbol *S, const MCExpr *Value) {
  auto *Symbol = cast<MCSymbolELF>(S);
  // If on rhs is micromaxis symbol then mark Symbol as microMaxis.
  if (Value->getKind() != MCExpr::SymbolRef)
    return;
  const auto &RhsSym = cast<MCSymbolELF>(
      static_cast<const MCSymbolRefExpr *>(Value)->getSymbol());

  if (!(RhsSym.getOther() & ELF::STO_MAXIS_MICROMAXIS))
    return;

  Symbol->setOther(ELF::STO_MAXIS_MICROMAXIS);
}

MCELFStreamer &MaxisTargetELFStreamer::getStreamer() {
  return static_cast<MCELFStreamer &>(Streamer);
}

void MaxisTargetELFStreamer::emitDirectiveSetMicroMaxis() {
  MicroMaxisEnabled = true;
  forbidModuleDirective();
}

void MaxisTargetELFStreamer::emitDirectiveSetNoMicroMaxis() {
  MicroMaxisEnabled = false;
  forbidModuleDirective();
}

void MaxisTargetELFStreamer::setUsesMicroMaxis() {
  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned Flags = MCA.getELFHeaderEFlags();
  Flags |= ELF::EF_MAXIS_MICROMAXIS;
  MCA.setELFHeaderEFlags(Flags);
}

void MaxisTargetELFStreamer::emitDirectiveSetMaxis16() {
  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned Flags = MCA.getELFHeaderEFlags();
  Flags |= ELF::EF_MAXIS_ARCH_ASE_M16;
  MCA.setELFHeaderEFlags(Flags);
  forbidModuleDirective();
}

void MaxisTargetELFStreamer::emitDirectiveSetNoReorder() {
  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned Flags = MCA.getELFHeaderEFlags();
  Flags |= ELF::EF_MAXIS_NOREORDER;
  MCA.setELFHeaderEFlags(Flags);
  forbidModuleDirective();
}

void MaxisTargetELFStreamer::emitDirectiveEnd(StringRef Name) {
  MCAssembler &MCA = getStreamer().getAssembler();
  MCContext &Context = MCA.getContext();
  MCStreamer &OS = getStreamer();

  MCSectionELF *Sec = Context.getELFSection(".pdr", ELF::SHT_PROGBITS, 0);

  MCSymbol *Sym = Context.getOrCreateSymbol(Name);
  const MCSymbolRefExpr *ExprRef =
      MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, Context);

  MCA.registerSection(*Sec);
  Sec->setAlignment(4);

  OS.PushSection();

  OS.SwitchSection(Sec);

  OS.EmitValueImpl(ExprRef, 4);

  OS.EmitIntValue(GPRInfoSet ? GPRBitMask : 0, 4); // reg_mask
  OS.EmitIntValue(GPRInfoSet ? GPROffset : 0, 4);  // reg_offset

  OS.EmitIntValue(FPRInfoSet ? FPRBitMask : 0, 4); // fpreg_mask
  OS.EmitIntValue(FPRInfoSet ? FPROffset : 0, 4);  // fpreg_offset

  OS.EmitIntValue(FrameInfoSet ? FrameOffset : 0, 4); // frame_offset
  OS.EmitIntValue(FrameInfoSet ? FrameReg : 0, 4);    // frame_reg
  OS.EmitIntValue(FrameInfoSet ? ReturnReg : 0, 4);   // return_reg

  // The .end directive marks the end of a procedure. Invalidate
  // the information gathered up until this point.
  GPRInfoSet = FPRInfoSet = FrameInfoSet = false;

  OS.PopSection();

  // .end also implicitly sets the size.
  MCSymbol *CurPCSym = Context.createTempSymbol();
  OS.EmitLabel(CurPCSym);
  const MCExpr *Size = MCBinaryExpr::createSub(
      MCSymbolRefExpr::create(CurPCSym, MCSymbolRefExpr::VK_None, Context),
      ExprRef, Context);

  // The ELFObjectWriter can determine the absolute size as it has access to
  // the layout information of the assembly file, so a size expression rather
  // than an absolute value is ok here.
  static_cast<MCSymbolELF *>(Sym)->setSize(Size);
}

void MaxisTargetELFStreamer::emitDirectiveEnt(const MCSymbol &Symbol) {
  GPRInfoSet = FPRInfoSet = FrameInfoSet = false;

  // .ent also acts like an implicit '.type symbol, STT_FUNC'
  static_cast<const MCSymbolELF &>(Symbol).setType(ELF::STT_FUNC);
}

void MaxisTargetELFStreamer::emitDirectiveAbiCalls() {
  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned Flags = MCA.getELFHeaderEFlags();
  Flags |= ELF::EF_MAXIS_CPIC | ELF::EF_MAXIS_PIC;
  MCA.setELFHeaderEFlags(Flags);
}

void MaxisTargetELFStreamer::emitDirectiveNaN2008() {
  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned Flags = MCA.getELFHeaderEFlags();
  Flags |= ELF::EF_MAXIS_NAN2008;
  MCA.setELFHeaderEFlags(Flags);
}

void MaxisTargetELFStreamer::emitDirectiveNaNLegacy() {
  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned Flags = MCA.getELFHeaderEFlags();
  Flags &= ~ELF::EF_MAXIS_NAN2008;
  MCA.setELFHeaderEFlags(Flags);
}

void MaxisTargetELFStreamer::emitDirectiveOptionPic0() {
  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned Flags = MCA.getELFHeaderEFlags();
  // This option overrides other PIC options like -KPIC.
  Pic = false;
  Flags &= ~ELF::EF_MAXIS_PIC;
  MCA.setELFHeaderEFlags(Flags);
}

void MaxisTargetELFStreamer::emitDirectiveOptionPic2() {
  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned Flags = MCA.getELFHeaderEFlags();
  Pic = true;
  // NOTE: We are following the GAS behaviour here which means the directive
  // 'pic2' also sets the CPIC bit in the ELF header. This is different from
  // what is stated in the SYSV ABI which consider the bits EF_MAXIS_PIC and
  // EF_MAXIS_CPIC to be mutually exclusive.
  Flags |= ELF::EF_MAXIS_PIC | ELF::EF_MAXIS_CPIC;
  MCA.setELFHeaderEFlags(Flags);
}

void MaxisTargetELFStreamer::emitDirectiveInsn() {
  MaxisTargetStreamer::emitDirectiveInsn();
  MaxisELFStreamer &MEF = static_cast<MaxisELFStreamer &>(Streamer);
  MEF.createPendingLabelRelocs();
}

void MaxisTargetELFStreamer::emitFrame(unsigned StackReg, unsigned StackSize,
                                      unsigned ReturnReg_) {
  MCContext &Context = getStreamer().getAssembler().getContext();
  const MCRegisterInfo *RegInfo = Context.getRegisterInfo();

  FrameInfoSet = true;
  FrameReg = RegInfo->getEncodingValue(StackReg);
  FrameOffset = StackSize;
  ReturnReg = RegInfo->getEncodingValue(ReturnReg_);
}

void MaxisTargetELFStreamer::emitMask(unsigned CPUBitmask,
                                     int CPUTopSavedRegOff) {
  GPRInfoSet = true;
  GPRBitMask = CPUBitmask;
  GPROffset = CPUTopSavedRegOff;
}

void MaxisTargetELFStreamer::emitFMask(unsigned FPUBitmask,
                                      int FPUTopSavedRegOff) {
  FPRInfoSet = true;
  FPRBitMask = FPUBitmask;
  FPROffset = FPUTopSavedRegOff;
}

void MaxisTargetELFStreamer::emitDirectiveCpLoad(unsigned RegNo) {
  // .cpload $reg
  // This directive expands to:
  // lui   $gp, %hi(_gp_disp)
  // addui $gp, $gp, %lo(_gp_disp)
  // addu  $gp, $gp, $reg
  // when support for position independent code is enabled.
  if (!Pic || (getABI().IsN32() || getABI().IsN64()))
    return;

  // There's a GNU extension controlled by -mno-shared that allows
  // locally-binding symbols to be accessed using absolute addresses.
  // This is currently not supported. When supported -mno-shared makes
  // .cpload expand to:
  //   lui     $gp, %hi(__gnu_local_gp)
  //   addiu   $gp, $gp, %lo(__gnu_local_gp)

  StringRef SymName("_gp_disp");
  MCAssembler &MCA = getStreamer().getAssembler();
  MCSymbol *GP_Disp = MCA.getContext().getOrCreateSymbol(SymName);
  MCA.registerSymbol(*GP_Disp);

  MCInst TmpInst;
  TmpInst.setOpcode(Maxis::LUi);
  TmpInst.addOperand(MCOperand::createReg(Maxis::GP));
  const MCExpr *HiSym = MaxisMCExpr::create(
      MaxisMCExpr::MEK_HI,
      MCSymbolRefExpr::create("_gp_disp", MCSymbolRefExpr::VK_None,
                              MCA.getContext()),
      MCA.getContext());
  TmpInst.addOperand(MCOperand::createExpr(HiSym));
  getStreamer().EmitInstruction(TmpInst, STI);

  TmpInst.clear();

  TmpInst.setOpcode(Maxis::ADDiu);
  TmpInst.addOperand(MCOperand::createReg(Maxis::GP));
  TmpInst.addOperand(MCOperand::createReg(Maxis::GP));
  const MCExpr *LoSym = MaxisMCExpr::create(
      MaxisMCExpr::MEK_LO,
      MCSymbolRefExpr::create("_gp_disp", MCSymbolRefExpr::VK_None,
                              MCA.getContext()),
      MCA.getContext());
  TmpInst.addOperand(MCOperand::createExpr(LoSym));
  getStreamer().EmitInstruction(TmpInst, STI);

  TmpInst.clear();

  TmpInst.setOpcode(Maxis::ADDu);
  TmpInst.addOperand(MCOperand::createReg(Maxis::GP));
  TmpInst.addOperand(MCOperand::createReg(Maxis::GP));
  TmpInst.addOperand(MCOperand::createReg(RegNo));
  getStreamer().EmitInstruction(TmpInst, STI);

  forbidModuleDirective();
}

bool MaxisTargetELFStreamer::emitDirectiveCpRestore(
    int Offset, function_ref<unsigned()> GetATReg, SMLoc IDLoc,
    const MCSubtargetInfo *STI) {
  MaxisTargetStreamer::emitDirectiveCpRestore(Offset, GetATReg, IDLoc, STI);
  // .cprestore offset
  // When PIC mode is enabled and the O32 ABI is used, this directive expands
  // to:
  //    sw $gp, offset($sp)
  // and adds a corresponding LW after every JAL.

  // Note that .cprestore is ignored if used with the N32 and N64 ABIs or if it
  // is used in non-PIC mode.
  if (!Pic || (getABI().IsN32() || getABI().IsN64()))
    return true;

  // Store the $gp on the stack.
  emitStoreWithImmOffset(Maxis::SW, Maxis::GP, Maxis::SP, Offset, GetATReg, IDLoc,
                         STI);
  return true;
}

void MaxisTargetELFStreamer::emitDirectiveCpsetup(unsigned RegNo,
                                                 int RegOrOffset,
                                                 const MCSymbol &Sym,
                                                 bool IsReg) {
  // Only N32 and N64 emit anything for .cpsetup iff PIC is set.
  if (!Pic || !(getABI().IsN32() || getABI().IsN64()))
    return;

  forbidModuleDirective();

  MCAssembler &MCA = getStreamer().getAssembler();
  MCInst Inst;

  // Either store the old $gp in a register or on the stack
  if (IsReg) {
    // move $save, $gpreg
    emitRRR(Maxis::OR64, RegOrOffset, Maxis::GP, Maxis::ZERO, SMLoc(), &STI);
  } else {
    // sd $gpreg, offset($sp)
    emitRRI(Maxis::SD, Maxis::GP, Maxis::SP, RegOrOffset, SMLoc(), &STI);
  }

  if (getABI().IsN32()) {
    MCSymbol *GPSym = MCA.getContext().getOrCreateSymbol("__gnu_local_gp");
    const MaxisMCExpr *HiExpr = MaxisMCExpr::create(
        MaxisMCExpr::MEK_HI, MCSymbolRefExpr::create(GPSym, MCA.getContext()),
        MCA.getContext());
    const MaxisMCExpr *LoExpr = MaxisMCExpr::create(
        MaxisMCExpr::MEK_LO, MCSymbolRefExpr::create(GPSym, MCA.getContext()),
        MCA.getContext());

    // lui $gp, %hi(__gnu_local_gp)
    emitRX(Maxis::LUi, Maxis::GP, MCOperand::createExpr(HiExpr), SMLoc(), &STI);

    // addiu  $gp, $gp, %lo(__gnu_local_gp)
    emitRRX(Maxis::ADDiu, Maxis::GP, Maxis::GP, MCOperand::createExpr(LoExpr),
            SMLoc(), &STI);

    return;
  }

  const MaxisMCExpr *HiExpr = MaxisMCExpr::createGpOff(
      MaxisMCExpr::MEK_HI, MCSymbolRefExpr::create(&Sym, MCA.getContext()),
      MCA.getContext());
  const MaxisMCExpr *LoExpr = MaxisMCExpr::createGpOff(
      MaxisMCExpr::MEK_LO, MCSymbolRefExpr::create(&Sym, MCA.getContext()),
      MCA.getContext());

  // lui $gp, %hi(%neg(%gp_rel(funcSym)))
  emitRX(Maxis::LUi, Maxis::GP, MCOperand::createExpr(HiExpr), SMLoc(), &STI);

  // addiu  $gp, $gp, %lo(%neg(%gp_rel(funcSym)))
  emitRRX(Maxis::ADDiu, Maxis::GP, Maxis::GP, MCOperand::createExpr(LoExpr),
          SMLoc(), &STI);

  // daddu  $gp, $gp, $funcreg
  emitRRR(Maxis::DADDu, Maxis::GP, Maxis::GP, RegNo, SMLoc(), &STI);
}

void MaxisTargetELFStreamer::emitDirectiveCpreturn(unsigned SaveLocation,
                                                  bool SaveLocationIsRegister) {
  // Only N32 and N64 emit anything for .cpreturn iff PIC is set.
  if (!Pic || !(getABI().IsN32() || getABI().IsN64()))
    return;

  MCInst Inst;
  // Either restore the old $gp from a register or on the stack
  if (SaveLocationIsRegister) {
    Inst.setOpcode(Maxis::OR);
    Inst.addOperand(MCOperand::createReg(Maxis::GP));
    Inst.addOperand(MCOperand::createReg(SaveLocation));
    Inst.addOperand(MCOperand::createReg(Maxis::ZERO));
  } else {
    Inst.setOpcode(Maxis::LD);
    Inst.addOperand(MCOperand::createReg(Maxis::GP));
    Inst.addOperand(MCOperand::createReg(Maxis::SP));
    Inst.addOperand(MCOperand::createImm(SaveLocation));
  }
  getStreamer().EmitInstruction(Inst, STI);

  forbidModuleDirective();
}

void MaxisTargetELFStreamer::emitMaxisAbiFlags() {
  MCAssembler &MCA = getStreamer().getAssembler();
  MCContext &Context = MCA.getContext();
  MCStreamer &OS = getStreamer();
  MCSectionELF *Sec = Context.getELFSection(
      ".MAXIS.abiflags", ELF::SHT_MAXIS_ABIFLAGS, ELF::SHF_ALLOC, 24, "");
  MCA.registerSection(*Sec);
  Sec->setAlignment(8);
  OS.SwitchSection(Sec);

  OS << ABIFlagsSection;
}
