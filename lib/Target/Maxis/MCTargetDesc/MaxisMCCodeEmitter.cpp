//===-- MaxisMCCodeEmitter.cpp - Convert Maxis Code to Machine Code ---------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the MaxisMCCodeEmitter class.
//
//===----------------------------------------------------------------------===//

#include "MaxisMCCodeEmitter.h"
#include "MCTargetDesc/MaxisFixupKinds.h"
#include "MCTargetDesc/MaxisMCExpr.h"
#include "MCTargetDesc/MaxisMCTargetDesc.h"
#include "llvm/ADT/APFloat.h"
#include "llvm/ADT/APInt.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include <cassert>
#include <cstdint>

using namespace llvm;

#define DEBUG_TYPE "mccodeemitter"

#define GET_INSTRMAP_INFO
#include "MaxisGenInstrInfo.inc"
#undef GET_INSTRMAP_INFO

namespace llvm {

MCCodeEmitter *createMaxisMCCodeEmitterEB(const MCInstrInfo &MCII,
                                         const MCRegisterInfo &MRI,
                                         MCContext &Ctx) {
  return new MaxisMCCodeEmitter(MCII, Ctx, false);
}

MCCodeEmitter *createMaxisMCCodeEmitterEL(const MCInstrInfo &MCII,
                                         const MCRegisterInfo &MRI,
                                         MCContext &Ctx) {
  return new MaxisMCCodeEmitter(MCII, Ctx, true);
}

} // end namespace llvm

// If the D<shift> instruction has a shift amount that is greater
// than 31 (checked in calling routine), lower it to a D<shift>32 instruction
static void LowerLargeShift(MCInst& Inst) {
  assert(Inst.getNumOperands() == 3 && "Invalid no. of operands for shift!");
  assert(Inst.getOperand(2).isImm());

  int64_t Shift = Inst.getOperand(2).getImm();
  if (Shift <= 31)
    return; // Do nothing
  Shift -= 32;

  // saminus32
  Inst.getOperand(2).setImm(Shift);

  switch (Inst.getOpcode()) {
  default:
    // Calling function is not synchronized
    llvm_unreachable("Unexpected shift instruction");
  case Maxis::DSLLi:
    Inst.setOpcode(Maxis::DSLLi32);
    return;
  case Maxis::DSRLi:
    Inst.setOpcode(Maxis::DSRLi32);
    return;
  case Maxis::DSRAi:
    Inst.setOpcode(Maxis::DSRAi32);
    return;
  case Maxis::DROTR:
    Inst.setOpcode(Maxis::DROTR32);
    return;
  }
}

// Fix a bad compact branch encoding for beqc/bnec.
void MaxisMCCodeEmitter::LowerCompactBranch(MCInst& Inst) const {
  // Encoding may be illegal !(rs < rt), but this situation is
  // easily fixed.
  unsigned RegOp0 = Inst.getOperand(0).getReg();
  unsigned RegOp1 = Inst.getOperand(1).getReg();

  unsigned Reg0 =  Ctx.getRegisterInfo()->getEncodingValue(RegOp0);
  unsigned Reg1 =  Ctx.getRegisterInfo()->getEncodingValue(RegOp1);

  if (Inst.getOpcode() == Maxis::BNEC || Inst.getOpcode() == Maxis::BEQC ||
      Inst.getOpcode() == Maxis::BNEC64 || Inst.getOpcode() == Maxis::BEQC64) {
    assert(Reg0 != Reg1 && "Instruction has bad operands ($rs == $rt)!");
    if (Reg0 < Reg1)
      return;
  } else if (Inst.getOpcode() == Maxis::BNVC || Inst.getOpcode() == Maxis::BOVC) {
    if (Reg0 >= Reg1)
      return;
  } else if (Inst.getOpcode() == Maxis::BNVC_MMR6 ||
             Inst.getOpcode() == Maxis::BOVC_MMR6) {
    if (Reg1 >= Reg0)
      return;
  } else
    llvm_unreachable("Cannot rewrite unknown branch!");

  Inst.getOperand(0).setReg(RegOp1);
  Inst.getOperand(1).setReg(RegOp0);
}

bool MaxisMCCodeEmitter::isMicroMaxis(const MCSubtargetInfo &STI) const {
  return STI.getFeatureBits()[Maxis::FeatureMicroMaxis];
}

bool MaxisMCCodeEmitter::isMaxis32r6(const MCSubtargetInfo &STI) const {
  return STI.getFeatureBits()[Maxis::FeatureMaxis32r6];
}

void MaxisMCCodeEmitter::EmitByte(unsigned char C, raw_ostream &OS) const {
  OS << (char)C;
}

void MaxisMCCodeEmitter::EmitInstruction(uint64_t Val, unsigned Size,
                                        const MCSubtargetInfo &STI,
                                        raw_ostream &OS) const {
  // Output the instruction encoding in little endian byte order.
  // Little-endian byte ordering:
  //   maxis32r2:   4 | 3 | 2 | 1
  //   microMAXIS:  2 | 1 | 4 | 3
  if (IsLittleEndian && Size == 4 && isMicroMaxis(STI)) {
    EmitInstruction(Val >> 16, 2, STI, OS);
    EmitInstruction(Val, 2, STI, OS);
  } else {
    for (unsigned i = 0; i < Size; ++i) {
      unsigned Shift = IsLittleEndian ? i * 8 : (Size - 1 - i) * 8;
      EmitByte((Val >> Shift) & 0xff, OS);
    }
  }
}

/// encodeInstruction - Emit the instruction.
/// Size the instruction with Desc.getSize().
void MaxisMCCodeEmitter::
encodeInstruction(const MCInst &MI, raw_ostream &OS,
                  SmallVectorImpl<MCFixup> &Fixups,
                  const MCSubtargetInfo &STI) const
{
  // Non-pseudo instructions that get changed for direct object
  // only based on operand values.
  // If this list of instructions get much longer we will move
  // the check to a function call. Until then, this is more efficient.
  MCInst TmpInst = MI;
  switch (MI.getOpcode()) {
  // If shift amount is >= 32 it the inst needs to be lowered further
  case Maxis::DSLLi:
  case Maxis::DSRLi:
  case Maxis::DSRAi:
  case Maxis::DROTR:
    LowerLargeShift(TmpInst);
    break;
  // Compact branches, enforce encoding restrictions.
  case Maxis::BEQC:
  case Maxis::BNEC:
  case Maxis::BEQC64:
  case Maxis::BNEC64:
  case Maxis::BOVC:
  case Maxis::BOVC_MMR6:
  case Maxis::BNVC:
  case Maxis::BNVC_MMR6:
    LowerCompactBranch(TmpInst);
  }

  unsigned long N = Fixups.size();
  uint32_t Binary = getBinaryCodeForInstr(TmpInst, Fixups, STI);

  // Check for unimplemented opcodes.
  // Unfortunately in MAXIS both NOP and SLLi will come in with Binary == 0
  // so we have to special check for them.
  unsigned Opcode = TmpInst.getOpcode();
  if ((Opcode != Maxis::NOP) && (Opcode != Maxis::SLLi) &&
      (Opcode != Maxis::SLLI_MM) && (Opcode != Maxis::SLLI_MMR6) && !Binary)
    llvm_unreachable("unimplemented opcode in encodeInstruction()");

  int NewOpcode = -1;
  if (isMicroMaxis(STI)) {
    if (isMaxis32r6(STI)) {
      NewOpcode = Maxis::MaxisR62MicroMaxisR6(Opcode, Maxis::Arch_micromaxisr6);
      if (NewOpcode == -1)
        NewOpcode = Maxis::Std2MicroMaxisR6(Opcode, Maxis::Arch_micromaxisr6);
    }
    else
      NewOpcode = Maxis::Std2MicroMaxis(Opcode, Maxis::Arch_micromaxis);

    // Check whether it is Dsp instruction.
    if (NewOpcode == -1)
      NewOpcode = Maxis::Dsp2MicroMaxis(Opcode, Maxis::Arch_mmdsp);

    if (NewOpcode != -1) {
      if (Fixups.size() > N)
        Fixups.pop_back();

      Opcode = NewOpcode;
      TmpInst.setOpcode (NewOpcode);
      Binary = getBinaryCodeForInstr(TmpInst, Fixups, STI);
    }
  }

  const MCInstrDesc &Desc = MCII.get(TmpInst.getOpcode());

  // Get byte count of instruction
  unsigned Size = Desc.getSize();
  if (!Size)
    llvm_unreachable("Desc.getSize() returns 0");

  EmitInstruction(Binary, Size, STI, OS);
}

/// getBranchTargetOpValue - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned MaxisMCCodeEmitter::
getBranchTargetOpValue(const MCInst &MI, unsigned OpNo,
                       SmallVectorImpl<MCFixup> &Fixups,
                       const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 4.
  if (MO.isImm()) return MO.getImm() >> 2;

  assert(MO.isExpr() &&
         "getBranchTargetOpValue expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-4, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(Maxis::fixup_Maxis_PC16)));
  return 0;
}

/// getBranchTargetOpValue1SImm16 - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned MaxisMCCodeEmitter::
getBranchTargetOpValue1SImm16(const MCInst &MI, unsigned OpNo,
                              SmallVectorImpl<MCFixup> &Fixups,
                              const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 2.
  if (MO.isImm()) return MO.getImm() >> 1;

  assert(MO.isExpr() &&
         "getBranchTargetOpValue expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-4, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(Maxis::fixup_Maxis_PC16)));
  return 0;
}

/// getBranchTargetOpValueMMR6 - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned MaxisMCCodeEmitter::
getBranchTargetOpValueMMR6(const MCInst &MI, unsigned OpNo,
                           SmallVectorImpl<MCFixup> &Fixups,
                           const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 2.
  if (MO.isImm())
    return MO.getImm() >> 1;

  assert(MO.isExpr() &&
         "getBranchTargetOpValueMMR6 expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-2, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(Maxis::fixup_Maxis_PC16)));
  return 0;
}

/// getBranchTargetOpValueLsl2MMR6 - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned MaxisMCCodeEmitter::
getBranchTargetOpValueLsl2MMR6(const MCInst &MI, unsigned OpNo,
                               SmallVectorImpl<MCFixup> &Fixups,
                               const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 4.
  if (MO.isImm())
    return MO.getImm() >> 2;

  assert(MO.isExpr() &&
         "getBranchTargetOpValueLsl2MMR6 expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-4, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(Maxis::fixup_Maxis_PC16)));
  return 0;
}

/// getBranchTarget7OpValueMM - Return binary encoding of the microMAXIS branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned MaxisMCCodeEmitter::
getBranchTarget7OpValueMM(const MCInst &MI, unsigned OpNo,
                          SmallVectorImpl<MCFixup> &Fixups,
                          const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 2.
  if (MO.isImm()) return MO.getImm() >> 1;

  assert(MO.isExpr() &&
         "getBranchTargetOpValueMM expects only expressions or immediates");

  const MCExpr *Expr = MO.getExpr();
  Fixups.push_back(MCFixup::create(0, Expr,
                                   MCFixupKind(Maxis::fixup_MICROMAXIS_PC7_S1)));
  return 0;
}

/// getBranchTargetOpValueMMPC10 - Return binary encoding of the microMAXIS
/// 10-bit branch target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned MaxisMCCodeEmitter::
getBranchTargetOpValueMMPC10(const MCInst &MI, unsigned OpNo,
                             SmallVectorImpl<MCFixup> &Fixups,
                             const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 2.
  if (MO.isImm()) return MO.getImm() >> 1;

  assert(MO.isExpr() &&
         "getBranchTargetOpValuePC10 expects only expressions or immediates");

  const MCExpr *Expr = MO.getExpr();
  Fixups.push_back(MCFixup::create(0, Expr,
                   MCFixupKind(Maxis::fixup_MICROMAXIS_PC10_S1)));
  return 0;
}

/// getBranchTargetOpValue - Return binary encoding of the microMAXIS branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned MaxisMCCodeEmitter::
getBranchTargetOpValueMM(const MCInst &MI, unsigned OpNo,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 2.
  if (MO.isImm()) return MO.getImm() >> 1;

  assert(MO.isExpr() &&
         "getBranchTargetOpValueMM expects only expressions or immediates");

  const MCExpr *Expr = MO.getExpr();
  Fixups.push_back(MCFixup::create(0, Expr,
                   MCFixupKind(Maxis::
                               fixup_MICROMAXIS_PC16_S1)));
  return 0;
}

/// getBranchTarget21OpValue - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned MaxisMCCodeEmitter::
getBranchTarget21OpValue(const MCInst &MI, unsigned OpNo,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 4.
  if (MO.isImm()) return MO.getImm() >> 2;

  assert(MO.isExpr() &&
         "getBranchTarget21OpValue expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-4, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(Maxis::fixup_MAXIS_PC21_S2)));
  return 0;
}

/// getBranchTarget21OpValueMM - Return binary encoding of the branch
/// target operand for microMAXIS. If the machine operand requires
/// relocation, record the relocation and return zero.
unsigned MaxisMCCodeEmitter::
getBranchTarget21OpValueMM(const MCInst &MI, unsigned OpNo,
                           SmallVectorImpl<MCFixup> &Fixups,
                           const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 4.
  if (MO.isImm()) return MO.getImm() >> 2;

  assert(MO.isExpr() &&
    "getBranchTarget21OpValueMM expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-4, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(Maxis::fixup_MICROMAXIS_PC21_S1)));
  return 0;
}

/// getBranchTarget26OpValue - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned MaxisMCCodeEmitter::
getBranchTarget26OpValue(const MCInst &MI, unsigned OpNo,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 4.
  if (MO.isImm()) return MO.getImm() >> 2;

  assert(MO.isExpr() &&
         "getBranchTarget26OpValue expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-4, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(Maxis::fixup_MAXIS_PC26_S2)));
  return 0;
}

/// getBranchTarget26OpValueMM - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned MaxisMCCodeEmitter::getBranchTarget26OpValueMM(
    const MCInst &MI, unsigned OpNo, SmallVectorImpl<MCFixup> &Fixups,
    const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 2.
  if (MO.isImm())
    return MO.getImm() >> 1;

  assert(MO.isExpr() &&
         "getBranchTarget26OpValueMM expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-4, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(Maxis::fixup_MICROMAXIS_PC26_S1)));
  return 0;
}

/// getJumpOffset16OpValue - Return binary encoding of the jump
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned MaxisMCCodeEmitter::
getJumpOffset16OpValue(const MCInst &MI, unsigned OpNo,
                       SmallVectorImpl<MCFixup> &Fixups,
                       const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  if (MO.isImm()) return MO.getImm();

  assert(MO.isExpr() &&
         "getJumpOffset16OpValue expects only expressions or an immediate");

   // TODO: Push fixup.
   return 0;
}

/// getJumpTargetOpValue - Return binary encoding of the jump
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned MaxisMCCodeEmitter::
getJumpTargetOpValue(const MCInst &MI, unsigned OpNo,
                     SmallVectorImpl<MCFixup> &Fixups,
                     const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  // If the destination is an immediate, divide by 4.
  if (MO.isImm()) return MO.getImm()>>2;

  assert(MO.isExpr() &&
         "getJumpTargetOpValue expects only expressions or an immediate");

  const MCExpr *Expr = MO.getExpr();
  Fixups.push_back(MCFixup::create(0, Expr,
                                   MCFixupKind(Maxis::fixup_Maxis_21)));
  return 0;
}

unsigned MaxisMCCodeEmitter::
getJumpTargetOpValueMM(const MCInst &MI, unsigned OpNo,
                       SmallVectorImpl<MCFixup> &Fixups,
                       const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  // If the destination is an immediate, divide by 2.
  if (MO.isImm()) return MO.getImm() >> 1;

  assert(MO.isExpr() &&
         "getJumpTargetOpValueMM expects only expressions or an immediate");

  const MCExpr *Expr = MO.getExpr();
  Fixups.push_back(MCFixup::create(0, Expr,
                                   MCFixupKind(Maxis::fixup_MICROMAXIS_26_S1)));
  return 0;
}

unsigned MaxisMCCodeEmitter::
getUImm5Lsl2Encoding(const MCInst &MI, unsigned OpNo,
                     SmallVectorImpl<MCFixup> &Fixups,
                     const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  if (MO.isImm()) {
    // The immediate is encoded as 'immediate << 2'.
    unsigned Res = getMachineOpValue(MI, MO, Fixups, STI);
    assert((Res & 3) == 0);
    return Res >> 2;
  }

  assert(MO.isExpr() &&
         "getUImm5Lsl2Encoding expects only expressions or an immediate");

  return 0;
}

unsigned MaxisMCCodeEmitter::
getSImm3Lsa2Value(const MCInst &MI, unsigned OpNo,
                  SmallVectorImpl<MCFixup> &Fixups,
                  const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  if (MO.isImm()) {
    int Value = MO.getImm();
    return Value >> 2;
  }

  return 0;
}

unsigned MaxisMCCodeEmitter::
getUImm6Lsl2Encoding(const MCInst &MI, unsigned OpNo,
                     SmallVectorImpl<MCFixup> &Fixups,
                     const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  if (MO.isImm()) {
    unsigned Value = MO.getImm();
    return Value >> 2;
  }

  return 0;
}

unsigned MaxisMCCodeEmitter::
getSImm9AddispValue(const MCInst &MI, unsigned OpNo,
                     SmallVectorImpl<MCFixup> &Fixups,
                     const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  if (MO.isImm()) {
    unsigned Binary = (MO.getImm() >> 2) & 0x0000ffff;
    return (((Binary & 0x8000) >> 7) | (Binary & 0x00ff));
  }

  return 0;
}

unsigned MaxisMCCodeEmitter::
getExprOpValue(const MCExpr *Expr, SmallVectorImpl<MCFixup> &Fixups,
               const MCSubtargetInfo &STI) const {
  int64_t Res;

  if (Expr->evaluateAsAbsolute(Res))
    return Res;

  MCExpr::ExprKind Kind = Expr->getKind();
  if (Kind == MCExpr::Constant) {
    return cast<MCConstantExpr>(Expr)->getValue();
  }

  if (Kind == MCExpr::Binary) {
    unsigned Res = getExprOpValue(cast<MCBinaryExpr>(Expr)->getLHS(), Fixups, STI);
    Res += getExprOpValue(cast<MCBinaryExpr>(Expr)->getRHS(), Fixups, STI);
    return Res;
  }

  if (Kind == MCExpr::Target) {
    const MaxisMCExpr *MaxisExpr = cast<MaxisMCExpr>(Expr);

    Maxis::Fixups FixupKind = Maxis::Fixups(0);
    switch (MaxisExpr->getKind()) {
    case MaxisMCExpr::MEK_None:
    case MaxisMCExpr::MEK_Special:
      llvm_unreachable("Unhandled fixup kind!");
      break;
    case MaxisMCExpr::MEK_CALL_HI16:
      FixupKind = Maxis::fixup_Maxis_CALL_HI16;
      break;
    case MaxisMCExpr::MEK_CALL_LO16:
      FixupKind = Maxis::fixup_Maxis_CALL_LO16;
      break;
    case MaxisMCExpr::MEK_DTPREL_HI:
      FixupKind = isMicroMaxis(STI) ? Maxis::fixup_MICROMAXIS_TLS_DTPREL_HI16
                                   : Maxis::fixup_Maxis_DTPREL_HI;
      break;
    case MaxisMCExpr::MEK_DTPREL_LO:
      FixupKind = isMicroMaxis(STI) ? Maxis::fixup_MICROMAXIS_TLS_DTPREL_LO16
                                   : Maxis::fixup_Maxis_DTPREL_LO;
      break;
    case MaxisMCExpr::MEK_GOTTPREL:
      FixupKind = isMicroMaxis(STI) ? Maxis::fixup_MICROMAXIS_GOTTPREL
                                   : Maxis::fixup_Maxis_GOTTPREL;
      break;
    case MaxisMCExpr::MEK_GOT:
      FixupKind = isMicroMaxis(STI) ? Maxis::fixup_MICROMAXIS_GOT16
                                   : Maxis::fixup_Maxis_GOT;
      break;
    case MaxisMCExpr::MEK_GOT_CALL:
      FixupKind = isMicroMaxis(STI) ? Maxis::fixup_MICROMAXIS_CALL16
                                   : Maxis::fixup_Maxis_CALL16;
      break;
    case MaxisMCExpr::MEK_GOT_DISP:
      FixupKind = isMicroMaxis(STI) ? Maxis::fixup_MICROMAXIS_GOT_DISP
                                   : Maxis::fixup_Maxis_GOT_DISP;
      break;
    case MaxisMCExpr::MEK_GOT_HI16:
      FixupKind = Maxis::fixup_Maxis_GOT_HI16;
      break;
    case MaxisMCExpr::MEK_GOT_LO16:
      FixupKind = Maxis::fixup_Maxis_GOT_LO16;
      break;
    case MaxisMCExpr::MEK_GOT_PAGE:
      FixupKind = isMicroMaxis(STI) ? Maxis::fixup_MICROMAXIS_GOT_PAGE
                                   : Maxis::fixup_Maxis_GOT_PAGE;
      break;
    case MaxisMCExpr::MEK_GOT_OFST:
      FixupKind = isMicroMaxis(STI) ? Maxis::fixup_MICROMAXIS_GOT_OFST
                                   : Maxis::fixup_Maxis_GOT_OFST;
      break;
    case MaxisMCExpr::MEK_GPREL:
      FixupKind = Maxis::fixup_Maxis_GPREL16;
      break;
    case MaxisMCExpr::MEK_LO:
      // Check for %lo(%neg(%gp_rel(X)))
      if (MaxisExpr->isGpOff()) {
        FixupKind = Maxis::fixup_Maxis_GPOFF_LO;
        break;
      }
      FixupKind = isMicroMaxis(STI) ? Maxis::fixup_MICROMAXIS_LO16
                                   : Maxis::fixup_Maxis_LO16;
      break;
    case MaxisMCExpr::MEK_HIGHEST:
      FixupKind = Maxis::fixup_Maxis_HIGHEST;
      break;
    case MaxisMCExpr::MEK_HIGHER:
      FixupKind = Maxis::fixup_Maxis_HIGHER;
      break;
    case MaxisMCExpr::MEK_HI:
      // Check for %hi(%neg(%gp_rel(X)))
      if (MaxisExpr->isGpOff()) {
        FixupKind = Maxis::fixup_Maxis_GPOFF_HI;
        break;
      }
      FixupKind = isMicroMaxis(STI) ? Maxis::fixup_MICROMAXIS_HI16
                                   : Maxis::fixup_Maxis_HI16;
      break;
    case MaxisMCExpr::MEK_PCREL_HI16:
      FixupKind = Maxis::fixup_MAXIS_PCHI16;
      break;
    case MaxisMCExpr::MEK_PCREL_LO16:
      FixupKind = Maxis::fixup_MAXIS_PCLO16;
      break;
    case MaxisMCExpr::MEK_TLSGD:
      FixupKind = isMicroMaxis(STI) ? Maxis::fixup_MICROMAXIS_TLS_GD
                                   : Maxis::fixup_Maxis_TLSGD;
      break;
    case MaxisMCExpr::MEK_TLSLDM:
      FixupKind = isMicroMaxis(STI) ? Maxis::fixup_MICROMAXIS_TLS_LDM
                                   : Maxis::fixup_Maxis_TLSLDM;
      break;
    case MaxisMCExpr::MEK_TPREL_HI:
      FixupKind = isMicroMaxis(STI) ? Maxis::fixup_MICROMAXIS_TLS_TPREL_HI16
                                   : Maxis::fixup_Maxis_TPREL_HI;
      break;
    case MaxisMCExpr::MEK_TPREL_LO:
      FixupKind = isMicroMaxis(STI) ? Maxis::fixup_MICROMAXIS_TLS_TPREL_LO16
                                   : Maxis::fixup_Maxis_TPREL_LO;
      break;
    case MaxisMCExpr::MEK_NEG:
      FixupKind =
          isMicroMaxis(STI) ? Maxis::fixup_MICROMAXIS_SUB : Maxis::fixup_Maxis_SUB;
      break;
    }
    Fixups.push_back(MCFixup::create(0, MaxisExpr, MCFixupKind(FixupKind)));
    return 0;
  }

  if (Kind == MCExpr::SymbolRef) {
    Maxis::Fixups FixupKind = Maxis::Fixups(0);

    switch(cast<MCSymbolRefExpr>(Expr)->getKind()) {
    default: llvm_unreachable("Unknown fixup kind!");
      break;
    case MCSymbolRefExpr::VK_None:
      FixupKind = Maxis::fixup_Maxis_32; // FIXME: This is ok for O32/N32 but not N64.
      break;
    } // switch

    Fixups.push_back(MCFixup::create(0, Expr, MCFixupKind(FixupKind)));
    return 0;
  }
  return 0;
}

/// getMachineOpValue - Return binary encoding of operand. If the machine
/// operand requires relocation, record the relocation and return zero.
unsigned MaxisMCCodeEmitter::
getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                  SmallVectorImpl<MCFixup> &Fixups,
                  const MCSubtargetInfo &STI) const {
  if (MO.isReg()) {
    unsigned Reg = MO.getReg();
    unsigned RegNo = Ctx.getRegisterInfo()->getEncodingValue(Reg);
    return RegNo;
  } else if (MO.isImm()) {
    return static_cast<unsigned>(MO.getImm());
  } else if (MO.isFPImm()) {
    return static_cast<unsigned>(APFloat(MO.getFPImm())
        .bitcastToAPInt().getHiBits(32).getLimitedValue());
  }
  // MO must be an Expr.
  assert(MO.isExpr());
  return getExprOpValue(MO.getExpr(),Fixups, STI);
}

/// Return binary encoding of memory related operand.
/// If the offset operand requires relocation, record the relocation.
template <unsigned ShiftAmount>
unsigned MaxisMCCodeEmitter::getMemEncoding(const MCInst &MI, unsigned OpNo,
                                           SmallVectorImpl<MCFixup> &Fixups,
                                           const MCSubtargetInfo &STI) const {
  // Base register is encoded in bits 20-16, offset is encoded in bits 15-0.
  assert(MI.getOperand(OpNo).isReg());
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo),Fixups, STI) << 16;
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1), Fixups, STI);

  // Apply the scale factor if there is one.
  OffBits >>= ShiftAmount;

  return (OffBits & 0xFFFF) | RegBits;
}

unsigned MaxisMCCodeEmitter::
getMemEncodingMMImm4(const MCInst &MI, unsigned OpNo,
                     SmallVectorImpl<MCFixup> &Fixups,
                     const MCSubtargetInfo &STI) const {
  // Base register is encoded in bits 6-4, offset is encoded in bits 3-0.
  assert(MI.getOperand(OpNo).isReg());
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo),
                                       Fixups, STI) << 4;
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1),
                                       Fixups, STI);

  return (OffBits & 0xF) | RegBits;
}

unsigned MaxisMCCodeEmitter::
getMemEncodingMMImm4Lsl1(const MCInst &MI, unsigned OpNo,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const {
  // Base register is encoded in bits 6-4, offset is encoded in bits 3-0.
  assert(MI.getOperand(OpNo).isReg());
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo),
                                       Fixups, STI) << 4;
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1),
                                       Fixups, STI) >> 1;

  return (OffBits & 0xF) | RegBits;
}

unsigned MaxisMCCodeEmitter::
getMemEncodingMMImm4Lsl2(const MCInst &MI, unsigned OpNo,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const {
  // Base register is encoded in bits 6-4, offset is encoded in bits 3-0.
  assert(MI.getOperand(OpNo).isReg());
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo),
                                       Fixups, STI) << 4;
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1),
                                       Fixups, STI) >> 2;

  return (OffBits & 0xF) | RegBits;
}

unsigned MaxisMCCodeEmitter::
getMemEncodingMMSPImm5Lsl2(const MCInst &MI, unsigned OpNo,
                           SmallVectorImpl<MCFixup> &Fixups,
                           const MCSubtargetInfo &STI) const {
  // Register is encoded in bits 9-5, offset is encoded in bits 4-0.
  assert(MI.getOperand(OpNo).isReg() &&
         (MI.getOperand(OpNo).getReg() == Maxis::SP ||
         MI.getOperand(OpNo).getReg() == Maxis::SP_64) &&
         "Unexpected base register!");
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1),
                                       Fixups, STI) >> 2;

  return OffBits & 0x1F;
}

unsigned MaxisMCCodeEmitter::
getMemEncodingMMGPImm7Lsl2(const MCInst &MI, unsigned OpNo,
                           SmallVectorImpl<MCFixup> &Fixups,
                           const MCSubtargetInfo &STI) const {
  // Register is encoded in bits 9-7, offset is encoded in bits 6-0.
  assert(MI.getOperand(OpNo).isReg() &&
         MI.getOperand(OpNo).getReg() == Maxis::GP &&
         "Unexpected base register!");

  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1),
                                       Fixups, STI) >> 2;

  return OffBits & 0x7F;
}

unsigned MaxisMCCodeEmitter::
getMemEncodingMMImm9(const MCInst &MI, unsigned OpNo,
                     SmallVectorImpl<MCFixup> &Fixups,
                     const MCSubtargetInfo &STI) const {
  // Base register is encoded in bits 20-16, offset is encoded in bits 8-0.
  assert(MI.getOperand(OpNo).isReg());
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups,
                                       STI) << 16;
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo + 1), Fixups, STI);

  return (OffBits & 0x1FF) | RegBits;
}

unsigned MaxisMCCodeEmitter::
getMemEncodingMMImm11(const MCInst &MI, unsigned OpNo,
                      SmallVectorImpl<MCFixup> &Fixups,
                      const MCSubtargetInfo &STI) const {
  // Base register is encoded in bits 20-16, offset is encoded in bits 10-0.
  assert(MI.getOperand(OpNo).isReg());
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups,
                                       STI) << 16;
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1), Fixups, STI);

  return (OffBits & 0x07FF) | RegBits;
}

unsigned MaxisMCCodeEmitter::
getMemEncodingMMImm12(const MCInst &MI, unsigned OpNo,
                      SmallVectorImpl<MCFixup> &Fixups,
                      const MCSubtargetInfo &STI) const {
  // opNum can be invalid if instruction had reglist as operand.
  // MemOperand is always last operand of instruction (base + offset).
  switch (MI.getOpcode()) {
  default:
    break;
  case Maxis::SWM32_MM:
  case Maxis::LWM32_MM:
    OpNo = MI.getNumOperands() - 2;
    break;
  }

  // Base register is encoded in bits 20-16, offset is encoded in bits 11-0.
  assert(MI.getOperand(OpNo).isReg());
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups, STI) << 16;
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1), Fixups, STI);

  return (OffBits & 0x0FFF) | RegBits;
}

unsigned MaxisMCCodeEmitter::
getMemEncodingMMImm16(const MCInst &MI, unsigned OpNo,
                      SmallVectorImpl<MCFixup> &Fixups,
                      const MCSubtargetInfo &STI) const {
  // Base register is encoded in bits 20-16, offset is encoded in bits 15-0.
  assert(MI.getOperand(OpNo).isReg());
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups,
                                       STI) << 16;
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1), Fixups, STI);

  return (OffBits & 0xFFFF) | RegBits;
}

unsigned MaxisMCCodeEmitter::
getMemEncodingMMImm4sp(const MCInst &MI, unsigned OpNo,
                       SmallVectorImpl<MCFixup> &Fixups,
                       const MCSubtargetInfo &STI) const {
  // opNum can be invalid if instruction had reglist as operand
  // MemOperand is always last operand of instruction (base + offset)
  switch (MI.getOpcode()) {
  default:
    break;
  case Maxis::SWM16_MM:
  case Maxis::SWM16_MMR6:
  case Maxis::LWM16_MM:
  case Maxis::LWM16_MMR6:
    OpNo = MI.getNumOperands() - 2;
    break;
  }

  // Offset is encoded in bits 4-0.
  assert(MI.getOperand(OpNo).isReg());
  // Base register is always SP - thus it is not encoded.
  assert(MI.getOperand(OpNo+1).isImm());
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1), Fixups, STI);

  return ((OffBits >> 2) & 0x0F);
}

// FIXME: should be called getMSBEncoding
//
unsigned
MaxisMCCodeEmitter::getSizeInsEncoding(const MCInst &MI, unsigned OpNo,
                                      SmallVectorImpl<MCFixup> &Fixups,
                                      const MCSubtargetInfo &STI) const {
  assert(MI.getOperand(OpNo-1).isImm());
  assert(MI.getOperand(OpNo).isImm());
  unsigned Position = getMachineOpValue(MI, MI.getOperand(OpNo-1), Fixups, STI);
  unsigned Size = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups, STI);

  return Position + Size - 1;
}

template <unsigned Bits, int Offset>
unsigned
MaxisMCCodeEmitter::getUImmWithOffsetEncoding(const MCInst &MI, unsigned OpNo,
                                             SmallVectorImpl<MCFixup> &Fixups,
                                             const MCSubtargetInfo &STI) const {
  assert(MI.getOperand(OpNo).isImm());
  unsigned Value = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups, STI);
  Value -= Offset;
  return Value;
}

unsigned
MaxisMCCodeEmitter::getSimm19Lsl2Encoding(const MCInst &MI, unsigned OpNo,
                                         SmallVectorImpl<MCFixup> &Fixups,
                                         const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  if (MO.isImm()) {
    // The immediate is encoded as 'immediate << 2'.
    unsigned Res = getMachineOpValue(MI, MO, Fixups, STI);
    assert((Res & 3) == 0);
    return Res >> 2;
  }

  assert(MO.isExpr() &&
         "getSimm19Lsl2Encoding expects only expressions or an immediate");

  const MCExpr *Expr = MO.getExpr();
  Maxis::Fixups FixupKind = isMicroMaxis(STI) ? Maxis::fixup_MICROMAXIS_PC19_S2
                                            : Maxis::fixup_MAXIS_PC19_S2;
  Fixups.push_back(MCFixup::create(0, Expr, MCFixupKind(FixupKind)));
  return 0;
}

unsigned
MaxisMCCodeEmitter::getSimm18Lsl3Encoding(const MCInst &MI, unsigned OpNo,
                                         SmallVectorImpl<MCFixup> &Fixups,
                                         const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  if (MO.isImm()) {
    // The immediate is encoded as 'immediate << 3'.
    unsigned Res = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups, STI);
    assert((Res & 7) == 0);
    return Res >> 3;
  }

  assert(MO.isExpr() &&
         "getSimm18Lsl2Encoding expects only expressions or an immediate");

  const MCExpr *Expr = MO.getExpr();
  Maxis::Fixups FixupKind = isMicroMaxis(STI) ? Maxis::fixup_MICROMAXIS_PC18_S3
                                            : Maxis::fixup_MAXIS_PC18_S3;
  Fixups.push_back(MCFixup::create(0, Expr, MCFixupKind(FixupKind)));
  return 0;
}

unsigned
MaxisMCCodeEmitter::getUImm3Mod8Encoding(const MCInst &MI, unsigned OpNo,
                                        SmallVectorImpl<MCFixup> &Fixups,
                                        const MCSubtargetInfo &STI) const {
  assert(MI.getOperand(OpNo).isImm());
  const MCOperand &MO = MI.getOperand(OpNo);
  return MO.getImm() % 8;
}

unsigned
MaxisMCCodeEmitter::getUImm4AndValue(const MCInst &MI, unsigned OpNo,
                                    SmallVectorImpl<MCFixup> &Fixups,
                                    const MCSubtargetInfo &STI) const {
  assert(MI.getOperand(OpNo).isImm());
  const MCOperand &MO = MI.getOperand(OpNo);
  unsigned Value = MO.getImm();
  switch (Value) {
    case 128:   return 0x0;
    case 1:     return 0x1;
    case 2:     return 0x2;
    case 3:     return 0x3;
    case 4:     return 0x4;
    case 7:     return 0x5;
    case 8:     return 0x6;
    case 15:    return 0x7;
    case 16:    return 0x8;
    case 31:    return 0x9;
    case 32:    return 0xa;
    case 63:    return 0xb;
    case 64:    return 0xc;
    case 255:   return 0xd;
    case 32768: return 0xe;
    case 65535: return 0xf;
  }
  llvm_unreachable("Unexpected value");
}

unsigned
MaxisMCCodeEmitter::getRegisterListOpValue(const MCInst &MI, unsigned OpNo,
                                          SmallVectorImpl<MCFixup> &Fixups,
                                          const MCSubtargetInfo &STI) const {
  unsigned res = 0;

  // Register list operand is always first operand of instruction and it is
  // placed before memory operand (register + imm).

  for (unsigned I = OpNo, E = MI.getNumOperands() - 2; I < E; ++I) {
    unsigned Reg = MI.getOperand(I).getReg();
    unsigned RegNo = Ctx.getRegisterInfo()->getEncodingValue(Reg);
    if (RegNo != 31)
      res++;
    else
      res |= 0x10;
  }
  return res;
}

unsigned
MaxisMCCodeEmitter::getRegisterListOpValue16(const MCInst &MI, unsigned OpNo,
                                            SmallVectorImpl<MCFixup> &Fixups,
                                            const MCSubtargetInfo &STI) const {
  return (MI.getNumOperands() - 4);
}

unsigned
MaxisMCCodeEmitter::getRegisterPairOpValue(const MCInst &MI, unsigned OpNo,
                                          SmallVectorImpl<MCFixup> &Fixups,
                                          const MCSubtargetInfo &STI) const {
  return getMachineOpValue(MI, MI.getOperand(OpNo), Fixups, STI);
}

unsigned
MaxisMCCodeEmitter::getMovePRegPairOpValue(const MCInst &MI, unsigned OpNo,
                                          SmallVectorImpl<MCFixup> &Fixups,
                                          const MCSubtargetInfo &STI) const {
  unsigned res = 0;

  if (MI.getOperand(0).getReg() == Maxis::A1 &&
      MI.getOperand(1).getReg() == Maxis::A2)
    res = 0;
  else if (MI.getOperand(0).getReg() == Maxis::A1 &&
           MI.getOperand(1).getReg() == Maxis::A3)
    res = 1;
  else if (MI.getOperand(0).getReg() == Maxis::A2 &&
           MI.getOperand(1).getReg() == Maxis::A3)
    res = 2;
  else if (MI.getOperand(0).getReg() == Maxis::A0 &&
           MI.getOperand(1).getReg() == Maxis::S5)
    res = 3;
  else if (MI.getOperand(0).getReg() == Maxis::A0 &&
           MI.getOperand(1).getReg() == Maxis::S6)
    res = 4;
  else if (MI.getOperand(0).getReg() == Maxis::A0 &&
           MI.getOperand(1).getReg() == Maxis::A1)
    res = 5;
  else if (MI.getOperand(0).getReg() == Maxis::A0 &&
           MI.getOperand(1).getReg() == Maxis::A2)
    res = 6;
  else if (MI.getOperand(0).getReg() == Maxis::A0 &&
           MI.getOperand(1).getReg() == Maxis::A3)
    res = 7;

  return res;
}

unsigned
MaxisMCCodeEmitter::getMovePRegSingleOpValue(const MCInst &MI, unsigned OpNo,
                                            SmallVectorImpl<MCFixup> &Fixups,
                                            const MCSubtargetInfo &STI) const {
  assert(((OpNo == 2) || (OpNo == 3)) &&
         "Unexpected OpNo for movep operand encoding!");

  MCOperand Op = MI.getOperand(OpNo);
  assert(Op.isReg() && "Operand of movep is not a register!");
  switch (Op.getReg()) {
  default:
    llvm_unreachable("Unknown register for movep!");
  case Maxis::ZERO:  return 0;
  case Maxis::S1:    return 1;
  case Maxis::V0:    return 2;
  case Maxis::V1:    return 3;
  case Maxis::S0:    return 4;
  case Maxis::S2:    return 5;
  case Maxis::S3:    return 6;
  case Maxis::S4:    return 7;
  }
}

unsigned
MaxisMCCodeEmitter::getSimm23Lsl2Encoding(const MCInst &MI, unsigned OpNo,
                                         SmallVectorImpl<MCFixup> &Fixups,
                                         const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  assert(MO.isImm() && "getSimm23Lsl2Encoding expects only an immediate");
  // The immediate is encoded as 'immediate >> 2'.
  unsigned Res = static_cast<unsigned>(MO.getImm());
  assert((Res & 3) == 0);
  return Res >> 2;
}

#include "MaxisGenMCCodeEmitter.inc"
