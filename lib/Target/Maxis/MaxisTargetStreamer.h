//===-- MaxisTargetStreamer.h - Maxis Target Streamer ------------*- C++ -*--===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MAXIS_MAXISTARGETSTREAMER_H
#define LLVM_LIB_TARGET_MAXIS_MAXISTARGETSTREAMER_H

#include "MCTargetDesc/MaxisABIFlagsSection.h"
#include "MCTargetDesc/MaxisABIInfo.h"
#include "llvm/ADT/Optional.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCStreamer.h"

namespace llvm {

struct MaxisABIFlagsSection;

class MaxisTargetStreamer : public MCTargetStreamer {
public:
  MaxisTargetStreamer(MCStreamer &S);

  virtual void setPic(bool Value) {}

  virtual void emitDirectiveSetMicroMaxis();
  virtual void emitDirectiveSetNoMicroMaxis();
  virtual void setUsesMicroMaxis();
  virtual void emitDirectiveSetMaxis16();
  virtual void emitDirectiveSetNoMaxis16();

  virtual void emitDirectiveSetReorder();
  virtual void emitDirectiveSetNoReorder();
  virtual void emitDirectiveSetMacro();
  virtual void emitDirectiveSetNoMacro();
  virtual void emitDirectiveSetMsa();
  virtual void emitDirectiveSetNoMsa();
  virtual void emitDirectiveSetMt();
  virtual void emitDirectiveSetNoMt();
  virtual void emitDirectiveSetAt();
  virtual void emitDirectiveSetAtWithArg(unsigned RegNo);
  virtual void emitDirectiveSetNoAt();
  virtual void emitDirectiveEnd(StringRef Name);

  virtual void emitDirectiveEnt(const MCSymbol &Symbol);
  virtual void emitDirectiveAbiCalls();
  virtual void emitDirectiveNaN2008();
  virtual void emitDirectiveNaNLegacy();
  virtual void emitDirectiveOptionPic0();
  virtual void emitDirectiveOptionPic2();
  virtual void emitDirectiveInsn();
  virtual void emitFrame(unsigned StackReg, unsigned StackSize,
                         unsigned ReturnReg);
  virtual void emitMask(unsigned CPUBitmask, int CPUTopSavedRegOff);
  virtual void emitFMask(unsigned FPUBitmask, int FPUTopSavedRegOff);

  virtual void emitDirectiveSetArch(StringRef Arch);
  virtual void emitDirectiveSetMaxis0();
  virtual void emitDirectiveSetMaxis1();
  virtual void emitDirectiveSetMaxis2();
  virtual void emitDirectiveSetMaxis3();
  virtual void emitDirectiveSetMaxis4();
  virtual void emitDirectiveSetMaxis5();
  virtual void emitDirectiveSetMaxis32();
  virtual void emitDirectiveSetMaxis32R2();
  virtual void emitDirectiveSetMaxis32R3();
  virtual void emitDirectiveSetMaxis32R5();
  virtual void emitDirectiveSetMaxis32R6();
  virtual void emitDirectiveSetMaxis64();
  virtual void emitDirectiveSetMaxis64R2();
  virtual void emitDirectiveSetMaxis64R3();
  virtual void emitDirectiveSetMaxis64R5();
  virtual void emitDirectiveSetMaxis64R6();
  virtual void emitDirectiveSetDsp();
  virtual void emitDirectiveSetDspr2();
  virtual void emitDirectiveSetNoDsp();
  virtual void emitDirectiveSetPop();
  virtual void emitDirectiveSetPush();
  virtual void emitDirectiveSetSoftFloat();
  virtual void emitDirectiveSetHardFloat();

  // PIC support
  virtual void emitDirectiveCpLoad(unsigned RegNo);
  virtual bool emitDirectiveCpRestore(int Offset,
                                      function_ref<unsigned()> GetATReg,
                                      SMLoc IDLoc, const MCSubtargetInfo *STI);
  virtual void emitDirectiveCpsetup(unsigned RegNo, int RegOrOffset,
                                    const MCSymbol &Sym, bool IsReg);
  virtual void emitDirectiveCpreturn(unsigned SaveLocation,
                                     bool SaveLocationIsRegister);

  // FP abiflags directives
  virtual void emitDirectiveModuleFP();
  virtual void emitDirectiveModuleOddSPReg();
  virtual void emitDirectiveModuleSoftFloat();
  virtual void emitDirectiveModuleHardFloat();
  virtual void emitDirectiveModuleMT();
  virtual void emitDirectiveSetFp(MaxisABIFlagsSection::FpABIKind Value);
  virtual void emitDirectiveSetOddSPReg();
  virtual void emitDirectiveSetNoOddSPReg();

  void emitR(unsigned Opcode, unsigned Reg0, SMLoc IDLoc,
             const MCSubtargetInfo *STI);
  void emitII(unsigned Opcode, int16_t Imm1, int16_t Imm2, SMLoc IDLoc,
              const MCSubtargetInfo *STI);
  void emitRX(unsigned Opcode, unsigned Reg0, MCOperand Op1, SMLoc IDLoc,
              const MCSubtargetInfo *STI);
  void emitRI(unsigned Opcode, unsigned Reg0, int32_t Imm, SMLoc IDLoc,
              const MCSubtargetInfo *STI);
  void emitRR(unsigned Opcode, unsigned Reg0, unsigned Reg1, SMLoc IDLoc,
              const MCSubtargetInfo *STI);
  void emitRRX(unsigned Opcode, unsigned Reg0, unsigned Reg1, MCOperand Op2,
               SMLoc IDLoc, const MCSubtargetInfo *STI);
  void emitRRR(unsigned Opcode, unsigned Reg0, unsigned Reg1, unsigned Reg2,
               SMLoc IDLoc, const MCSubtargetInfo *STI);
  void emitRRI(unsigned Opcode, unsigned Reg0, unsigned Reg1, int16_t Imm,
               SMLoc IDLoc, const MCSubtargetInfo *STI);
  void emitRRIII(unsigned Opcode, unsigned Reg0, unsigned Reg1, int16_t Imm0,
                 int16_t Imm1, int16_t Imm2, SMLoc IDLoc,
                 const MCSubtargetInfo *STI);
  void emitAddu(unsigned DstReg, unsigned SrcReg, unsigned TrgReg, bool Is64Bit,
                const MCSubtargetInfo *STI);
  void emitDSLL(unsigned DstReg, unsigned SrcReg, int16_t ShiftAmount,
                SMLoc IDLoc, const MCSubtargetInfo *STI);
  void emitEmptyDelaySlot(bool hasShortDelaySlot, SMLoc IDLoc,
                          const MCSubtargetInfo *STI);
  void emitNop(SMLoc IDLoc, const MCSubtargetInfo *STI);

  /// Emit a store instruction with an offset. If the offset is out of range
  /// then it will be synthesized using the assembler temporary.
  ///
  /// GetATReg() is a callback that can be used to obtain the current assembler
  /// temporary and is only called when the assembler temporary is required. It
  /// must handle the case where no assembler temporary is available (typically
  /// by reporting an error).
  void emitStoreWithImmOffset(unsigned Opcode, unsigned SrcReg,
                              unsigned BaseReg, int64_t Offset,
                              function_ref<unsigned()> GetATReg, SMLoc IDLoc,
                              const MCSubtargetInfo *STI);
  void emitStoreWithSymOffset(unsigned Opcode, unsigned SrcReg,
                              unsigned BaseReg, MCOperand &HiOperand,
                              MCOperand &LoOperand, unsigned ATReg, SMLoc IDLoc,
                              const MCSubtargetInfo *STI);
  void emitLoadWithImmOffset(unsigned Opcode, unsigned DstReg, unsigned BaseReg,
                             int64_t Offset, unsigned TmpReg, SMLoc IDLoc,
                             const MCSubtargetInfo *STI);
  void emitLoadWithSymOffset(unsigned Opcode, unsigned DstReg, unsigned BaseReg,
                             MCOperand &HiOperand, MCOperand &LoOperand,
                             unsigned ATReg, SMLoc IDLoc,
                             const MCSubtargetInfo *STI);
  void emitGPRestore(int Offset, SMLoc IDLoc, const MCSubtargetInfo *STI);

  void forbidModuleDirective() { ModuleDirectiveAllowed = false; }
  void reallowModuleDirective() { ModuleDirectiveAllowed = true; }
  bool isModuleDirectiveAllowed() { return ModuleDirectiveAllowed; }

  // This method enables template classes to set internal abi flags
  // structure values.
  template <class PredicateLibrary>
  void updateABIInfo(const PredicateLibrary &P) {
    ABI = P.getABI();
    ABIFlagsSection.setAllFromPredicates(P);
  }

  MaxisABIFlagsSection &getABIFlagsSection() { return ABIFlagsSection; }
  const MaxisABIInfo &getABI() const {
    assert(ABI.hasValue() && "ABI hasn't been set!");
    return *ABI;
  }

protected:
  llvm::Optional<MaxisABIInfo> ABI;
  MaxisABIFlagsSection ABIFlagsSection;

  bool GPRInfoSet;
  unsigned GPRBitMask;
  int GPROffset;

  bool FPRInfoSet;
  unsigned FPRBitMask;
  int FPROffset;

  bool FrameInfoSet;
  int FrameOffset;
  unsigned FrameReg;
  unsigned ReturnReg;

private:
  bool ModuleDirectiveAllowed;
};

// This part is for ascii assembly output
class MaxisTargetAsmStreamer : public MaxisTargetStreamer {
  formatted_raw_ostream &OS;

public:
  MaxisTargetAsmStreamer(MCStreamer &S, formatted_raw_ostream &OS);
  void emitDirectiveSetMicroMaxis() override;
  void emitDirectiveSetNoMicroMaxis() override;
  void emitDirectiveSetMaxis16() override;
  void emitDirectiveSetNoMaxis16() override;

  void emitDirectiveSetReorder() override;
  void emitDirectiveSetNoReorder() override;
  void emitDirectiveSetMacro() override;
  void emitDirectiveSetNoMacro() override;
  void emitDirectiveSetMsa() override;
  void emitDirectiveSetNoMsa() override;
  void emitDirectiveSetMt() override;
  void emitDirectiveSetNoMt() override;
  void emitDirectiveSetAt() override;
  void emitDirectiveSetAtWithArg(unsigned RegNo) override;
  void emitDirectiveSetNoAt() override;
  void emitDirectiveEnd(StringRef Name) override;

  void emitDirectiveEnt(const MCSymbol &Symbol) override;
  void emitDirectiveAbiCalls() override;
  void emitDirectiveNaN2008() override;
  void emitDirectiveNaNLegacy() override;
  void emitDirectiveOptionPic0() override;
  void emitDirectiveOptionPic2() override;
  void emitDirectiveInsn() override;
  void emitFrame(unsigned StackReg, unsigned StackSize,
                 unsigned ReturnReg) override;
  void emitMask(unsigned CPUBitmask, int CPUTopSavedRegOff) override;
  void emitFMask(unsigned FPUBitmask, int FPUTopSavedRegOff) override;

  void emitDirectiveSetArch(StringRef Arch) override;
  void emitDirectiveSetMaxis0() override;
  void emitDirectiveSetMaxis1() override;
  void emitDirectiveSetMaxis2() override;
  void emitDirectiveSetMaxis3() override;
  void emitDirectiveSetMaxis4() override;
  void emitDirectiveSetMaxis5() override;
  void emitDirectiveSetMaxis32() override;
  void emitDirectiveSetMaxis32R2() override;
  void emitDirectiveSetMaxis32R3() override;
  void emitDirectiveSetMaxis32R5() override;
  void emitDirectiveSetMaxis32R6() override;
  void emitDirectiveSetMaxis64() override;
  void emitDirectiveSetMaxis64R2() override;
  void emitDirectiveSetMaxis64R3() override;
  void emitDirectiveSetMaxis64R5() override;
  void emitDirectiveSetMaxis64R6() override;
  void emitDirectiveSetDsp() override;
  void emitDirectiveSetDspr2() override;
  void emitDirectiveSetNoDsp() override;
  void emitDirectiveSetPop() override;
  void emitDirectiveSetPush() override;
  void emitDirectiveSetSoftFloat() override;
  void emitDirectiveSetHardFloat() override;

  // PIC support
  void emitDirectiveCpLoad(unsigned RegNo) override;

  /// Emit a .cprestore directive.  If the offset is out of range then it will
  /// be synthesized using the assembler temporary.
  ///
  /// GetATReg() is a callback that can be used to obtain the current assembler
  /// temporary and is only called when the assembler temporary is required. It
  /// must handle the case where no assembler temporary is available (typically
  /// by reporting an error).
  bool emitDirectiveCpRestore(int Offset, function_ref<unsigned()> GetATReg,
                              SMLoc IDLoc, const MCSubtargetInfo *STI) override;
  void emitDirectiveCpsetup(unsigned RegNo, int RegOrOffset,
                            const MCSymbol &Sym, bool IsReg) override;
  void emitDirectiveCpreturn(unsigned SaveLocation,
                             bool SaveLocationIsRegister) override;

  // FP abiflags directives
  void emitDirectiveModuleFP() override;
  void emitDirectiveModuleOddSPReg() override;
  void emitDirectiveModuleSoftFloat() override;
  void emitDirectiveModuleHardFloat() override;
  void emitDirectiveModuleMT() override;
  void emitDirectiveSetFp(MaxisABIFlagsSection::FpABIKind Value) override;
  void emitDirectiveSetOddSPReg() override;
  void emitDirectiveSetNoOddSPReg() override;
};

// This part is for ELF object output
class MaxisTargetELFStreamer : public MaxisTargetStreamer {
  bool MicroMaxisEnabled;
  const MCSubtargetInfo &STI;
  bool Pic;

public:
  bool isMicroMaxisEnabled() const { return MicroMaxisEnabled; }
  MCELFStreamer &getStreamer();
  MaxisTargetELFStreamer(MCStreamer &S, const MCSubtargetInfo &STI);

  void setPic(bool Value) override { Pic = Value; }

  void emitLabel(MCSymbol *Symbol) override;
  void emitAssignment(MCSymbol *Symbol, const MCExpr *Value) override;
  void finish() override;

  void emitDirectiveSetMicroMaxis() override;
  void emitDirectiveSetNoMicroMaxis() override;
  void setUsesMicroMaxis() override;
  void emitDirectiveSetMaxis16() override;

  void emitDirectiveSetNoReorder() override;
  void emitDirectiveEnd(StringRef Name) override;

  void emitDirectiveEnt(const MCSymbol &Symbol) override;
  void emitDirectiveAbiCalls() override;
  void emitDirectiveNaN2008() override;
  void emitDirectiveNaNLegacy() override;
  void emitDirectiveOptionPic0() override;
  void emitDirectiveOptionPic2() override;
  void emitDirectiveInsn() override;
  void emitFrame(unsigned StackReg, unsigned StackSize,
                 unsigned ReturnReg) override;
  void emitMask(unsigned CPUBitmask, int CPUTopSavedRegOff) override;
  void emitFMask(unsigned FPUBitmask, int FPUTopSavedRegOff) override;

  // PIC support
  void emitDirectiveCpLoad(unsigned RegNo) override;
  bool emitDirectiveCpRestore(int Offset, function_ref<unsigned()> GetATReg,
                              SMLoc IDLoc, const MCSubtargetInfo *STI) override;
  void emitDirectiveCpsetup(unsigned RegNo, int RegOrOffset,
                            const MCSymbol &Sym, bool IsReg) override;
  void emitDirectiveCpreturn(unsigned SaveLocation,
                             bool SaveLocationIsRegister) override;

  void emitMaxisAbiFlags();
};
}
#endif
