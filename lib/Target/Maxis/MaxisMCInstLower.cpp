//===- MaxisMCInstLower.cpp - Convert Maxis MachineInstr to MCInst ----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains code to lower Maxis MachineInstrs to their corresponding
// MCInst records.
//
//===----------------------------------------------------------------------===//

#include "MaxisMCInstLower.h"
#include "MCTargetDesc/MaxisBaseInfo.h"
#include "MCTargetDesc/MaxisMCExpr.h"
#include "MaxisAsmPrinter.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/Support/ErrorHandling.h"
#include <cassert>

using namespace llvm;

MaxisMCInstLower::MaxisMCInstLower(MaxisAsmPrinter &asmprinter)
  : AsmPrinter(asmprinter) {}

void MaxisMCInstLower::Initialize(MCContext *C) {
  Ctx = C;
}

MCOperand MaxisMCInstLower::LowerSymbolOperand(const MachineOperand &MO,
                                              MachineOperandType MOTy,
                                              unsigned Offset) const {
  MCSymbolRefExpr::VariantKind Kind = MCSymbolRefExpr::VK_None;
  MaxisMCExpr::MaxisExprKind TargetKind = MaxisMCExpr::MEK_None;
  bool IsGpOff = false;
  const MCSymbol *Symbol;

  switch(MO.getTargetFlags()) {
  default:
    llvm_unreachable("Invalid target flag!");
  case MaxisII::MO_NO_FLAG:
    break;
  case MaxisII::MO_GPREL:
    TargetKind = MaxisMCExpr::MEK_GPREL;
    break;
  case MaxisII::MO_GOT_CALL:
    TargetKind = MaxisMCExpr::MEK_GOT_CALL;
    break;
  case MaxisII::MO_GOT:
    TargetKind = MaxisMCExpr::MEK_GOT;
    break;
  case MaxisII::MO_ABS_HI:
    TargetKind = MaxisMCExpr::MEK_HI;
    break;
  case MaxisII::MO_ABS_LO:
    TargetKind = MaxisMCExpr::MEK_LO;
    break;
  case MaxisII::MO_TLSGD:
    TargetKind = MaxisMCExpr::MEK_TLSGD;
    break;
  case MaxisII::MO_TLSLDM:
    TargetKind = MaxisMCExpr::MEK_TLSLDM;
    break;
  case MaxisII::MO_DTPREL_HI:
    TargetKind = MaxisMCExpr::MEK_DTPREL_HI;
    break;
  case MaxisII::MO_DTPREL_LO:
    TargetKind = MaxisMCExpr::MEK_DTPREL_LO;
    break;
  case MaxisII::MO_GOTTPREL:
    TargetKind = MaxisMCExpr::MEK_GOTTPREL;
    break;
  case MaxisII::MO_TPREL_HI:
    TargetKind = MaxisMCExpr::MEK_TPREL_HI;
    break;
  case MaxisII::MO_TPREL_LO:
    TargetKind = MaxisMCExpr::MEK_TPREL_LO;
    break;
  case MaxisII::MO_GPOFF_HI:
    TargetKind = MaxisMCExpr::MEK_HI;
    IsGpOff = true;
    break;
  case MaxisII::MO_GPOFF_LO:
    TargetKind = MaxisMCExpr::MEK_LO;
    IsGpOff = true;
    break;
  case MaxisII::MO_GOT_DISP:
    TargetKind = MaxisMCExpr::MEK_GOT_DISP;
    break;
  case MaxisII::MO_GOT_HI16:
    TargetKind = MaxisMCExpr::MEK_GOT_HI16;
    break;
  case MaxisII::MO_GOT_LO16:
    TargetKind = MaxisMCExpr::MEK_GOT_LO16;
    break;
  case MaxisII::MO_GOT_PAGE:
    TargetKind = MaxisMCExpr::MEK_GOT_PAGE;
    break;
  case MaxisII::MO_GOT_OFST:
    TargetKind = MaxisMCExpr::MEK_GOT_OFST;
    break;
  case MaxisII::MO_HIGHER:
    TargetKind = MaxisMCExpr::MEK_HIGHER;
    break;
  case MaxisII::MO_HIGHEST:
    TargetKind = MaxisMCExpr::MEK_HIGHEST;
    break;
  case MaxisII::MO_CALL_HI16:
    TargetKind = MaxisMCExpr::MEK_CALL_HI16;
    break;
  case MaxisII::MO_CALL_LO16:
    TargetKind = MaxisMCExpr::MEK_CALL_LO16;
    break;
  }

  switch (MOTy) {
  case MachineOperand::MO_MachineBasicBlock:
    Symbol = MO.getMBB()->getSymbol();
    break;

  case MachineOperand::MO_GlobalAddress:
    Symbol = AsmPrinter.getSymbol(MO.getGlobal());
    Offset += MO.getOffset();
    break;

  case MachineOperand::MO_BlockAddress:
    Symbol = AsmPrinter.GetBlockAddressSymbol(MO.getBlockAddress());
    Offset += MO.getOffset();
    break;

  case MachineOperand::MO_ExternalSymbol:
    Symbol = AsmPrinter.GetExternalSymbolSymbol(MO.getSymbolName());
    Offset += MO.getOffset();
    break;

  case MachineOperand::MO_MCSymbol:
    Symbol = MO.getMCSymbol();
    Offset += MO.getOffset();
    break;

  case MachineOperand::MO_JumpTableIndex:
    Symbol = AsmPrinter.GetJTISymbol(MO.getIndex());
    break;

  case MachineOperand::MO_ConstantPoolIndex:
    Symbol = AsmPrinter.GetCPISymbol(MO.getIndex());
    Offset += MO.getOffset();
    break;

  default:
    llvm_unreachable("<unknown operand type>");
  }

  const MCExpr *Expr = MCSymbolRefExpr::create(Symbol, Kind, *Ctx);

  if (Offset) {
    // Assume offset is never negative.
    assert(Offset > 0);

    Expr = MCBinaryExpr::createAdd(Expr, MCConstantExpr::create(Offset, *Ctx),
                                   *Ctx);
  }

  if (IsGpOff)
    Expr = MaxisMCExpr::createGpOff(TargetKind, Expr, *Ctx);
  else if (TargetKind != MaxisMCExpr::MEK_None)
    Expr = MaxisMCExpr::create(TargetKind, Expr, *Ctx);

  return MCOperand::createExpr(Expr);
}

MCOperand MaxisMCInstLower::LowerOperand(const MachineOperand &MO,
                                        unsigned offset) const {
  MachineOperandType MOTy = MO.getType();

  switch (MOTy) {
  default: llvm_unreachable("unknown operand type");
  case MachineOperand::MO_Register:
    // Ignore all implicit register operands.
    if (MO.isImplicit()) break;
    return MCOperand::createReg(MO.getReg());
  case MachineOperand::MO_Immediate:
    return MCOperand::createImm(MO.getImm() + offset);
  case MachineOperand::MO_MachineBasicBlock:
  case MachineOperand::MO_GlobalAddress:
  case MachineOperand::MO_ExternalSymbol:
  case MachineOperand::MO_MCSymbol:
  case MachineOperand::MO_JumpTableIndex:
  case MachineOperand::MO_ConstantPoolIndex:
  case MachineOperand::MO_BlockAddress:
    return LowerSymbolOperand(MO, MOTy, offset);
  case MachineOperand::MO_RegisterMask:
    break;
 }

  return MCOperand();
}

MCOperand MaxisMCInstLower::createSub(MachineBasicBlock *BB1,
                                     MachineBasicBlock *BB2,
                                     MaxisMCExpr::MaxisExprKind Kind) const {
  const MCSymbolRefExpr *Sym1 = MCSymbolRefExpr::create(BB1->getSymbol(), *Ctx);
  const MCSymbolRefExpr *Sym2 = MCSymbolRefExpr::create(BB2->getSymbol(), *Ctx);
  const MCBinaryExpr *Sub = MCBinaryExpr::createSub(Sym1, Sym2, *Ctx);

  return MCOperand::createExpr(MaxisMCExpr::create(Kind, Sub, *Ctx));
}

void MaxisMCInstLower::
lowerLongBranchLUi(const MachineInstr *MI, MCInst &OutMI) const {
  OutMI.setOpcode(Maxis::LUi);

  // Lower register operand.
  OutMI.addOperand(LowerOperand(MI->getOperand(0)));

  // Create %hi($tgt-$baltgt).
  OutMI.addOperand(createSub(MI->getOperand(1).getMBB(),
                             MI->getOperand(2).getMBB(),
                             MaxisMCExpr::MEK_HI));
}

void MaxisMCInstLower::lowerLongBranchADDiu(
    const MachineInstr *MI, MCInst &OutMI, int Opcode,
    MaxisMCExpr::MaxisExprKind Kind) const {
  OutMI.setOpcode(Opcode);

  // Lower two register operands.
  for (unsigned I = 0, E = 2; I != E; ++I) {
    const MachineOperand &MO = MI->getOperand(I);
    OutMI.addOperand(LowerOperand(MO));
  }

  // Create %lo($tgt-$baltgt) or %hi($tgt-$baltgt).
  OutMI.addOperand(createSub(MI->getOperand(2).getMBB(),
                             MI->getOperand(3).getMBB(), Kind));
}

bool MaxisMCInstLower::lowerLongBranch(const MachineInstr *MI,
                                      MCInst &OutMI) const {
  switch (MI->getOpcode()) {
  default:
    return false;
  case Maxis::LONG_BRANCH_LUi:
    lowerLongBranchLUi(MI, OutMI);
    return true;
  case Maxis::LONG_BRANCH_ADDiu:
    lowerLongBranchADDiu(MI, OutMI, Maxis::ADDiu, MaxisMCExpr::MEK_LO);
    return true;
  case Maxis::LONG_BRANCH_DADDiu:
    unsigned TargetFlags = MI->getOperand(2).getTargetFlags();
    if (TargetFlags == MaxisII::MO_ABS_HI)
      lowerLongBranchADDiu(MI, OutMI, Maxis::DADDiu, MaxisMCExpr::MEK_HI);
    else if (TargetFlags == MaxisII::MO_ABS_LO)
      lowerLongBranchADDiu(MI, OutMI, Maxis::DADDiu, MaxisMCExpr::MEK_LO);
    else
      report_fatal_error("Unexpected flags for LONG_BRANCH_DADDiu");
    return true;
  }
}

void MaxisMCInstLower::Lower(const MachineInstr *MI, MCInst &OutMI) const {
  if (lowerLongBranch(MI, OutMI))
    return;

  OutMI.setOpcode(MI->getOpcode());

  for (unsigned i = 0, e = MI->getNumOperands(); i != e; ++i) {
    const MachineOperand &MO = MI->getOperand(i);
    MCOperand MCOp = LowerOperand(MO);

    if (MCOp.isValid())
      OutMI.addOperand(MCOp);
  }
}
