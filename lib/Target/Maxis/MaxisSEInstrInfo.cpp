//===-- MaxisSEInstrInfo.cpp - Maxis32/64 Instruction Information -----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the Maxis32/64 implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "MaxisSEInstrInfo.h"
#include "InstPrinter/MaxisInstPrinter.h"
#include "MaxisAnalyzeImmediate.h"
#include "MaxisMachineFunction.h"
#include "MaxisTargetMachine.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

MaxisSEInstrInfo::MaxisSEInstrInfo(const MaxisSubtarget &STI)
    : MaxisInstrInfo(STI, STI.isPositionIndependent() ? Maxis::B : Maxis::J),
      RI() {}

const MaxisRegisterInfo &MaxisSEInstrInfo::getRegisterInfo() const {
  return RI;
}

/// isLoadFromStackSlot - If the specified machine instruction is a direct
/// load from a stack slot, return the virtual or physical register number of
/// the destination along with the FrameIndex of the loaded stack slot.  If
/// not, return 0.  This predicate must return 0 if the instruction has
/// any side effects other than loading from the stack slot.
unsigned MaxisSEInstrInfo::isLoadFromStackSlot(const MachineInstr &MI,
                                              int &FrameIndex) const {
  unsigned Opc = MI.getOpcode();

  if ((Opc == Maxis::LW)   || (Opc == Maxis::LD)   ||
      (Opc == Maxis::LWC1) || (Opc == Maxis::LDC1) || (Opc == Maxis::LDC164)) {
    if ((MI.getOperand(1).isFI()) &&  // is a stack slot
        (MI.getOperand(2).isImm()) && // the imm is zero
        (isZeroImm(MI.getOperand(2)))) {
      FrameIndex = MI.getOperand(1).getIndex();
      return MI.getOperand(0).getReg();
    }
  }

  return 0;
}

/// isStoreToStackSlot - If the specified machine instruction is a direct
/// store to a stack slot, return the virtual or physical register number of
/// the source reg along with the FrameIndex of the loaded stack slot.  If
/// not, return 0.  This predicate must return 0 if the instruction has
/// any side effects other than storing to the stack slot.
unsigned MaxisSEInstrInfo::isStoreToStackSlot(const MachineInstr &MI,
                                             int &FrameIndex) const {
  unsigned Opc = MI.getOpcode();

  if ((Opc == Maxis::SW)   || (Opc == Maxis::SD)   ||
      (Opc == Maxis::SWC1) || (Opc == Maxis::SDC1) || (Opc == Maxis::SDC164)) {
    if ((MI.getOperand(1).isFI()) &&  // is a stack slot
        (MI.getOperand(2).isImm()) && // the imm is zero
        (isZeroImm(MI.getOperand(2)))) {
      FrameIndex = MI.getOperand(1).getIndex();
      return MI.getOperand(0).getReg();
    }
  }
  return 0;
}

void MaxisSEInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                                  MachineBasicBlock::iterator I,
                                  const DebugLoc &DL, unsigned DestReg,
                                  unsigned SrcReg, bool KillSrc) const {
  unsigned Opc = 0, ZeroReg = 0;
  bool isMicroMaxis = Subtarget.inMicroMaxisMode();

  if (Maxis::GPR32RegClass.contains(DestReg)) { // Copy to CPU Reg.
    if (Maxis::GPR32RegClass.contains(SrcReg)) {
      if (isMicroMaxis)
        Opc = Maxis::MOVE16_MM;
      else
        Opc = Maxis::OR, ZeroReg = Maxis::ZERO;
    } else if (Maxis::CCRRegClass.contains(SrcReg))
      Opc = Maxis::CFC1;
    else if (Maxis::FGR32RegClass.contains(SrcReg))
      Opc = Maxis::MFC1;
    else if (Maxis::HI32RegClass.contains(SrcReg)) {
      Opc = isMicroMaxis ? Maxis::MFHI16_MM : Maxis::MFHI;
      SrcReg = 0;
    } else if (Maxis::LO32RegClass.contains(SrcReg)) {
      Opc = isMicroMaxis ? Maxis::MFLO16_MM : Maxis::MFLO;
      SrcReg = 0;
    } else if (Maxis::HI32DSPRegClass.contains(SrcReg))
      Opc = Maxis::MFHI_DSP;
    else if (Maxis::LO32DSPRegClass.contains(SrcReg))
      Opc = Maxis::MFLO_DSP;
    else if (Maxis::DSPCCRegClass.contains(SrcReg)) {
      BuildMI(MBB, I, DL, get(Maxis::RDDSP), DestReg).addImm(1 << 4)
        .addReg(SrcReg, RegState::Implicit | getKillRegState(KillSrc));
      return;
    }
    else if (Maxis::MSACtrlRegClass.contains(SrcReg))
      Opc = Maxis::CFCMSA;
  }
  else if (Maxis::GPR32RegClass.contains(SrcReg)) { // Copy from CPU Reg.
    if (Maxis::CCRRegClass.contains(DestReg))
      Opc = Maxis::CTC1;
    else if (Maxis::FGR32RegClass.contains(DestReg))
      Opc = Maxis::MTC1;
    else if (Maxis::HI32RegClass.contains(DestReg))
      Opc = Maxis::MTHI, DestReg = 0;
    else if (Maxis::LO32RegClass.contains(DestReg))
      Opc = Maxis::MTLO, DestReg = 0;
    else if (Maxis::HI32DSPRegClass.contains(DestReg))
      Opc = Maxis::MTHI_DSP;
    else if (Maxis::LO32DSPRegClass.contains(DestReg))
      Opc = Maxis::MTLO_DSP;
    else if (Maxis::DSPCCRegClass.contains(DestReg)) {
      BuildMI(MBB, I, DL, get(Maxis::WRDSP))
        .addReg(SrcReg, getKillRegState(KillSrc)).addImm(1 << 4)
        .addReg(DestReg, RegState::ImplicitDefine);
      return;
    } else if (Maxis::MSACtrlRegClass.contains(DestReg)) {
      BuildMI(MBB, I, DL, get(Maxis::CTCMSA))
          .addReg(DestReg)
          .addReg(SrcReg, getKillRegState(KillSrc));
      return;
    }
  }
  else if (Maxis::FGR32RegClass.contains(DestReg, SrcReg))
    Opc = Maxis::FMOV_S;
  else if (Maxis::AFGR64RegClass.contains(DestReg, SrcReg))
    Opc = Maxis::FMOV_D32;
  else if (Maxis::FGR64RegClass.contains(DestReg, SrcReg))
    Opc = Maxis::FMOV_D64;
  else if (Maxis::GPR64RegClass.contains(DestReg)) { // Copy to CPU64 Reg.
    if (Maxis::GPR64RegClass.contains(SrcReg))
      Opc = Maxis::OR64, ZeroReg = Maxis::ZERO_64;
    else if (Maxis::HI64RegClass.contains(SrcReg))
      Opc = Maxis::MFHI64, SrcReg = 0;
    else if (Maxis::LO64RegClass.contains(SrcReg))
      Opc = Maxis::MFLO64, SrcReg = 0;
    else if (Maxis::FGR64RegClass.contains(SrcReg))
      Opc = Maxis::DMFC1;
  }
  else if (Maxis::GPR64RegClass.contains(SrcReg)) { // Copy from CPU64 Reg.
    if (Maxis::HI64RegClass.contains(DestReg))
      Opc = Maxis::MTHI64, DestReg = 0;
    else if (Maxis::LO64RegClass.contains(DestReg))
      Opc = Maxis::MTLO64, DestReg = 0;
    else if (Maxis::FGR64RegClass.contains(DestReg))
      Opc = Maxis::DMTC1;
  }
  else if (Maxis::MSA128BRegClass.contains(DestReg)) { // Copy to MSA reg
    if (Maxis::MSA128BRegClass.contains(SrcReg))
      Opc = Maxis::MOVE_V;
  }

  assert(Opc && "Cannot copy registers");

  MachineInstrBuilder MIB = BuildMI(MBB, I, DL, get(Opc));

  if (DestReg)
    MIB.addReg(DestReg, RegState::Define);

  if (SrcReg)
    MIB.addReg(SrcReg, getKillRegState(KillSrc));

  if (ZeroReg)
    MIB.addReg(ZeroReg);
}

void MaxisSEInstrInfo::
storeRegToStack(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                unsigned SrcReg, bool isKill, int FI,
                const TargetRegisterClass *RC, const TargetRegisterInfo *TRI,
                int64_t Offset) const {
  DebugLoc DL;
  MachineMemOperand *MMO = GetMemOperand(MBB, FI, MachineMemOperand::MOStore);

  unsigned Opc = 0;

  if (Maxis::GPR32RegClass.hasSubClassEq(RC))
    Opc = Maxis::SW;
  else if (Maxis::GPR64RegClass.hasSubClassEq(RC))
    Opc = Maxis::SD;
  else if (Maxis::ACC64RegClass.hasSubClassEq(RC))
    Opc = Maxis::STORE_ACC64;
  else if (Maxis::ACC64DSPRegClass.hasSubClassEq(RC))
    Opc = Maxis::STORE_ACC64DSP;
  else if (Maxis::ACC128RegClass.hasSubClassEq(RC))
    Opc = Maxis::STORE_ACC128;
  else if (Maxis::DSPCCRegClass.hasSubClassEq(RC))
    Opc = Maxis::STORE_CCOND_DSP;
  else if (Maxis::FGR32RegClass.hasSubClassEq(RC))
    Opc = Maxis::SWC1;
  else if (Maxis::AFGR64RegClass.hasSubClassEq(RC))
    Opc = Maxis::SDC1;
  else if (Maxis::FGR64RegClass.hasSubClassEq(RC))
    Opc = Maxis::SDC164;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v16i8))
    Opc = Maxis::ST_B;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v8i16) ||
           TRI->isTypeLegalForClass(*RC, MVT::v8f16))
    Opc = Maxis::ST_H;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v4i32) ||
           TRI->isTypeLegalForClass(*RC, MVT::v4f32))
    Opc = Maxis::ST_W;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v2i64) ||
           TRI->isTypeLegalForClass(*RC, MVT::v2f64))
    Opc = Maxis::ST_D;
  else if (Maxis::LO32RegClass.hasSubClassEq(RC))
    Opc = Maxis::SW;
  else if (Maxis::LO64RegClass.hasSubClassEq(RC))
    Opc = Maxis::SD;
  else if (Maxis::HI32RegClass.hasSubClassEq(RC))
    Opc = Maxis::SW;
  else if (Maxis::HI64RegClass.hasSubClassEq(RC))
    Opc = Maxis::SD;
  else if (Maxis::DSPRRegClass.hasSubClassEq(RC))
    Opc = Maxis::SWDSP;

  // Hi, Lo are normally caller save but they are callee save
  // for interrupt handling.
  const Function &Func = MBB.getParent()->getFunction();
  if (Func.hasFnAttribute("interrupt")) {
    if (Maxis::HI32RegClass.hasSubClassEq(RC)) {
      BuildMI(MBB, I, DL, get(Maxis::MFHI), Maxis::K0);
      SrcReg = Maxis::K0;
    } else if (Maxis::HI64RegClass.hasSubClassEq(RC)) {
      BuildMI(MBB, I, DL, get(Maxis::MFHI64), Maxis::K0_64);
      SrcReg = Maxis::K0_64;
    } else if (Maxis::LO32RegClass.hasSubClassEq(RC)) {
      BuildMI(MBB, I, DL, get(Maxis::MFLO), Maxis::K0);
      SrcReg = Maxis::K0;
    } else if (Maxis::LO64RegClass.hasSubClassEq(RC)) {
      BuildMI(MBB, I, DL, get(Maxis::MFLO64), Maxis::K0_64);
      SrcReg = Maxis::K0_64;
    }
  }

  assert(Opc && "Register class not handled!");
  BuildMI(MBB, I, DL, get(Opc)).addReg(SrcReg, getKillRegState(isKill))
    .addFrameIndex(FI).addImm(Offset).addMemOperand(MMO);
}

void MaxisSEInstrInfo::
loadRegFromStack(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                 unsigned DestReg, int FI, const TargetRegisterClass *RC,
                 const TargetRegisterInfo *TRI, int64_t Offset) const {
  DebugLoc DL;
  if (I != MBB.end()) DL = I->getDebugLoc();
  MachineMemOperand *MMO = GetMemOperand(MBB, FI, MachineMemOperand::MOLoad);
  unsigned Opc = 0;

  const Function &Func = MBB.getParent()->getFunction();
  bool ReqIndirectLoad = Func.hasFnAttribute("interrupt") &&
                         (DestReg == Maxis::LO0 || DestReg == Maxis::LO0_64 ||
                          DestReg == Maxis::HI0 || DestReg == Maxis::HI0_64);

  if (Maxis::GPR32RegClass.hasSubClassEq(RC))
    Opc = Maxis::LW;
  else if (Maxis::GPR64RegClass.hasSubClassEq(RC))
    Opc = Maxis::LD;
  else if (Maxis::ACC64RegClass.hasSubClassEq(RC))
    Opc = Maxis::LOAD_ACC64;
  else if (Maxis::ACC64DSPRegClass.hasSubClassEq(RC))
    Opc = Maxis::LOAD_ACC64DSP;
  else if (Maxis::ACC128RegClass.hasSubClassEq(RC))
    Opc = Maxis::LOAD_ACC128;
  else if (Maxis::DSPCCRegClass.hasSubClassEq(RC))
    Opc = Maxis::LOAD_CCOND_DSP;
  else if (Maxis::FGR32RegClass.hasSubClassEq(RC))
    Opc = Maxis::LWC1;
  else if (Maxis::AFGR64RegClass.hasSubClassEq(RC))
    Opc = Maxis::LDC1;
  else if (Maxis::FGR64RegClass.hasSubClassEq(RC))
    Opc = Maxis::LDC164;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v16i8))
    Opc = Maxis::LD_B;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v8i16) ||
           TRI->isTypeLegalForClass(*RC, MVT::v8f16))
    Opc = Maxis::LD_H;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v4i32) ||
           TRI->isTypeLegalForClass(*RC, MVT::v4f32))
    Opc = Maxis::LD_W;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v2i64) ||
           TRI->isTypeLegalForClass(*RC, MVT::v2f64))
    Opc = Maxis::LD_D;
  else if (Maxis::HI32RegClass.hasSubClassEq(RC))
    Opc = Maxis::LW;
  else if (Maxis::HI64RegClass.hasSubClassEq(RC))
    Opc = Maxis::LD;
  else if (Maxis::LO32RegClass.hasSubClassEq(RC))
    Opc = Maxis::LW;
  else if (Maxis::LO64RegClass.hasSubClassEq(RC))
    Opc = Maxis::LD;
  else if (Maxis::DSPRRegClass.hasSubClassEq(RC))
    Opc = Maxis::LWDSP;

  assert(Opc && "Register class not handled!");

  if (!ReqIndirectLoad)
    BuildMI(MBB, I, DL, get(Opc), DestReg)
        .addFrameIndex(FI)
        .addImm(Offset)
        .addMemOperand(MMO);
  else {
    // Load HI/LO through K0. Notably the DestReg is encoded into the
    // instruction itself.
    unsigned Reg = Maxis::K0;
    unsigned LdOp = Maxis::MTLO;
    if (DestReg == Maxis::HI0)
      LdOp = Maxis::MTHI;

    if (Subtarget.getABI().ArePtrs64bit()) {
      Reg = Maxis::K0_64;
      if (DestReg == Maxis::HI0_64)
        LdOp = Maxis::MTHI64;
      else
        LdOp = Maxis::MTLO64;
    }

    BuildMI(MBB, I, DL, get(Opc), Reg)
        .addFrameIndex(FI)
        .addImm(Offset)
        .addMemOperand(MMO);
    BuildMI(MBB, I, DL, get(LdOp)).addReg(Reg);
  }
}

bool MaxisSEInstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
  MachineBasicBlock &MBB = *MI.getParent();
  bool isMicroMaxis = Subtarget.inMicroMaxisMode();
  unsigned Opc;

  switch (MI.getDesc().getOpcode()) {
  default:
    return false;
  case Maxis::RetRA:
    expandRetRA(MBB, MI);
    break;
  case Maxis::ERet:
    expandERet(MBB, MI);
    break;
  case Maxis::PseudoMFHI:
    Opc = isMicroMaxis ? Maxis::MFHI16_MM : Maxis::MFHI;
    expandPseudoMFHiLo(MBB, MI, Opc);
    break;
  case Maxis::PseudoMFLO:
    Opc = isMicroMaxis ? Maxis::MFLO16_MM : Maxis::MFLO;
    expandPseudoMFHiLo(MBB, MI, Opc);
    break;
  case Maxis::PseudoMFHI64:
    expandPseudoMFHiLo(MBB, MI, Maxis::MFHI64);
    break;
  case Maxis::PseudoMFLO64:
    expandPseudoMFHiLo(MBB, MI, Maxis::MFLO64);
    break;
  case Maxis::PseudoMTLOHI:
    expandPseudoMTLoHi(MBB, MI, Maxis::MTLO, Maxis::MTHI, false);
    break;
  case Maxis::PseudoMTLOHI64:
    expandPseudoMTLoHi(MBB, MI, Maxis::MTLO64, Maxis::MTHI64, false);
    break;
  case Maxis::PseudoMTLOHI_DSP:
    expandPseudoMTLoHi(MBB, MI, Maxis::MTLO_DSP, Maxis::MTHI_DSP, true);
    break;
  case Maxis::PseudoCVT_S_W:
    expandCvtFPInt(MBB, MI, Maxis::CVT_S_W, Maxis::MTC1, false);
    break;
  case Maxis::PseudoCVT_D32_W:
    expandCvtFPInt(MBB, MI, Maxis::CVT_D32_W, Maxis::MTC1, false);
    break;
  case Maxis::PseudoCVT_S_L:
    expandCvtFPInt(MBB, MI, Maxis::CVT_S_L, Maxis::DMTC1, true);
    break;
  case Maxis::PseudoCVT_D64_W:
    expandCvtFPInt(MBB, MI, Maxis::CVT_D64_W, Maxis::MTC1, true);
    break;
  case Maxis::PseudoCVT_D64_L:
    expandCvtFPInt(MBB, MI, Maxis::CVT_D64_L, Maxis::DMTC1, true);
    break;
  case Maxis::BuildPairF64:
    expandBuildPairF64(MBB, MI, false);
    break;
  case Maxis::BuildPairF64_64:
    expandBuildPairF64(MBB, MI, true);
    break;
  case Maxis::ExtractElementF64:
    expandExtractElementF64(MBB, MI, false);
    break;
  case Maxis::ExtractElementF64_64:
    expandExtractElementF64(MBB, MI, true);
    break;
  case Maxis::MAXISeh_return32:
  case Maxis::MAXISeh_return64:
    expandEhReturn(MBB, MI);
    break;
  }

  MBB.erase(MI);
  return true;
}

/// getOppositeBranchOpc - Return the inverse of the specified
/// opcode, e.g. turning BEQ to BNE.
unsigned MaxisSEInstrInfo::getOppositeBranchOpc(unsigned Opc) const {
  switch (Opc) {
  default:           llvm_unreachable("Illegal opcode!");
  case Maxis::BEQ:    return Maxis::BNE;
  case Maxis::BEQ_MM: return Maxis::BNE_MM;
  case Maxis::BNE:    return Maxis::BEQ;
  case Maxis::BNE_MM: return Maxis::BEQ_MM;
  case Maxis::BGTZ:   return Maxis::BLEZ;
  case Maxis::BGEZ:   return Maxis::BLTZ;
  case Maxis::BLTZ:   return Maxis::BGEZ;
  case Maxis::BLT:    return Maxis::BGE;
  case Maxis::BGE:    return Maxis::BLT;
  case Maxis::BLEZ:   return Maxis::BGTZ;
  case Maxis::BEQ64:  return Maxis::BNE64;
  case Maxis::BNE64:  return Maxis::BEQ64;
  case Maxis::BGTZ64: return Maxis::BLEZ64;
  case Maxis::BGEZ64: return Maxis::BLTZ64;
  case Maxis::BLTZ64: return Maxis::BGEZ64;
  case Maxis::BLEZ64: return Maxis::BGTZ64;
  case Maxis::BC1T:   return Maxis::BC1F;
  case Maxis::BC1F:   return Maxis::BC1T;
  case Maxis::BEQZC_MM: return Maxis::BNEZC_MM;
  case Maxis::BNEZC_MM: return Maxis::BEQZC_MM;
  case Maxis::BEQZC:  return Maxis::BNEZC;
  case Maxis::BNEZC:  return Maxis::BEQZC;
  case Maxis::BEQC:   return Maxis::BNEC;
  case Maxis::BNEC:   return Maxis::BEQC;
  case Maxis::BGTZC:  return Maxis::BLEZC;
  case Maxis::BGEZC:  return Maxis::BLTZC;
  case Maxis::BLTZC:  return Maxis::BGEZC;
  case Maxis::BLEZC:  return Maxis::BGTZC;
  case Maxis::BEQZC64:  return Maxis::BNEZC64;
  case Maxis::BNEZC64:  return Maxis::BEQZC64;
  case Maxis::BEQC64:   return Maxis::BNEC64;
  case Maxis::BNEC64:   return Maxis::BEQC64;
  case Maxis::BGEC64:   return Maxis::BLTC64;
  case Maxis::BGEUC64:  return Maxis::BLTUC64;
  case Maxis::BLTC64:   return Maxis::BGEC64;
  case Maxis::BLTUC64:  return Maxis::BGEUC64;
  case Maxis::BGTZC64:  return Maxis::BLEZC64;
  case Maxis::BGEZC64:  return Maxis::BLTZC64;
  case Maxis::BLTZC64:  return Maxis::BGEZC64;
  case Maxis::BLEZC64:  return Maxis::BGTZC64;
  case Maxis::BBIT0:  return Maxis::BBIT1;
  case Maxis::BBIT1:  return Maxis::BBIT0;
  case Maxis::BBIT032:  return Maxis::BBIT132;
  case Maxis::BBIT132:  return Maxis::BBIT032;
  }
}

/// Adjust SP by Amount bytes.
void MaxisSEInstrInfo::adjustStackPtr(unsigned SP, int64_t Amount,
                                     MachineBasicBlock &MBB,
                                     MachineBasicBlock::iterator I) const {
  MaxisABIInfo ABI = Subtarget.getABI();
  DebugLoc DL;
  unsigned ADDi = ABI.GetPtrAddiOp();

  if (Amount == 0)
    return;

  if (isInt<16>(Amount)) {
    // addi sp, sp, amount
    BuildMI(MBB, I, DL, get(ADDi), SP).addReg(SP).addImm(Amount);
  } else {
    // For numbers which are not 16bit integers we synthesize Amount inline
    // then add or subtract it from sp.
    unsigned Opc = ABI.GetPtrAdduOp();
    if (Amount < 0) {
      Opc = ABI.GetPtrSubuOp();
      Amount = -Amount;
    }
    unsigned Reg = loadImmediate(Amount, MBB, I, DL, nullptr);
    BuildMI(MBB, I, DL, get(Opc), SP).addReg(SP).addReg(Reg, RegState::Kill);
  }
}

/// This function generates the sequence of instructions needed to get the
/// result of adding register REG and immediate IMM.
unsigned MaxisSEInstrInfo::loadImmediate(int64_t Imm, MachineBasicBlock &MBB,
                                        MachineBasicBlock::iterator II,
                                        const DebugLoc &DL,
                                        unsigned *NewImm) const {
  MaxisAnalyzeImmediate AnalyzeImm;
  const MaxisSubtarget &STI = Subtarget;
  MachineRegisterInfo &RegInfo = MBB.getParent()->getRegInfo();
  unsigned Size = STI.isABI_N64() ? 64 : 32;
  unsigned CATi = STI.isABI_N64() ? Maxis::CATi64 : Maxis::CATi;
  unsigned ZeroReg = STI.isABI_N64() ? Maxis::ZERO_64 : Maxis::ZERO;
  const TargetRegisterClass *RC = STI.isABI_N64() ?
    &Maxis::GPR64RegClass : &Maxis::GPR32RegClass;
  bool LastInstrIsADDi = NewImm;

  const MaxisAnalyzeImmediate::InstSeq &Seq =
    AnalyzeImm.Analyze(Imm, Size, LastInstrIsADDi);
  MaxisAnalyzeImmediate::InstSeq::const_iterator Inst = Seq.begin();

  assert(Seq.size() && (!LastInstrIsADDi || (Seq.size() > 1)));

  // The first instruction can be a CATi, which is different from other
  // instructions (ADDi, ORI and SLLi) in that it does not have a register
  // operand.
  unsigned Reg = RegInfo.createVirtualRegister(RC);

  if (Inst->Opc == CATi)
    BuildMI(MBB, II, DL, get(CATi), Reg).addReg(ZeroReg).addImm(SignExtend64<16>(Inst->ImmOpnd));
  else
    BuildMI(MBB, II, DL, get(Inst->Opc), Reg).addReg(ZeroReg)
      .addImm(SignExtend64<16>(Inst->ImmOpnd));

  // Build the remaining instructions in Seq.
  for (++Inst; Inst != Seq.end() - LastInstrIsADDi; ++Inst)
    BuildMI(MBB, II, DL, get(Inst->Opc), Reg).addReg(Reg, RegState::Kill)
      .addImm(SignExtend64<16>(Inst->ImmOpnd));

  if (LastInstrIsADDi)
    *NewImm = Inst->ImmOpnd;

  return Reg;
}

unsigned MaxisSEInstrInfo::getAnalyzableBrOpc(unsigned Opc) const {
  return (Opc == Maxis::BEQ    || Opc == Maxis::BEQ_MM || Opc == Maxis::BNE    ||
          Opc == Maxis::BNE_MM || Opc == Maxis::BGTZ   || Opc == Maxis::BGEZ   ||
          Opc == Maxis::BLT    || Opc == Maxis::BGE  ||
          Opc == Maxis::BLTZ   || Opc == Maxis::BLEZ   || Opc == Maxis::BEQ64  ||
          Opc == Maxis::BNE64  || Opc == Maxis::BGTZ64 || Opc == Maxis::BGEZ64 ||
          Opc == Maxis::BLTZ64 || Opc == Maxis::BLEZ64 || Opc == Maxis::BC1T   ||
          Opc == Maxis::BC1F   || Opc == Maxis::B      || Opc == Maxis::J      ||
          Opc == Maxis::B_MM   || Opc == Maxis::BEQZC_MM ||
          Opc == Maxis::BNEZC_MM || Opc == Maxis::BEQC || Opc == Maxis::BNEC   ||
          Opc == Maxis::BLTC   || Opc == Maxis::BGEC   || Opc == Maxis::BLTUC  ||
          Opc == Maxis::BGEUC  || Opc == Maxis::BGTZC  || Opc == Maxis::BLEZC  ||
          Opc == Maxis::BGEZC  || Opc == Maxis::BLTZC  || Opc == Maxis::BEQZC  ||
          Opc == Maxis::BNEZC  || Opc == Maxis::BEQZC64 || Opc == Maxis::BNEZC64 ||
          Opc == Maxis::BEQC64 || Opc == Maxis::BNEC64 || Opc == Maxis::BGEC64 ||
          Opc == Maxis::BGEUC64 || Opc == Maxis::BLTC64 || Opc == Maxis::BLTUC64 ||
          Opc == Maxis::BGTZC64 || Opc == Maxis::BGEZC64 ||
          Opc == Maxis::BLTZC64 || Opc == Maxis::BLEZC64 || Opc == Maxis::BC ||
          Opc == Maxis::BBIT0 || Opc == Maxis::BBIT1 || Opc == Maxis::BBIT032 ||
          Opc == Maxis::BBIT132) ? Opc : 0;
}

void MaxisSEInstrInfo::expandRetRA(MachineBasicBlock &MBB,
                                  MachineBasicBlock::iterator I) const {

  MachineInstrBuilder MIB;
  if (Subtarget.isGP64bit())
    MIB = BuildMI(MBB, I, I->getDebugLoc(), get(Maxis::PseudoReturn64))
              .addReg(Maxis::RA_64, RegState::Undef);
  else
    MIB = BuildMI(MBB, I, I->getDebugLoc(), get(Maxis::PseudoReturn))
              .addReg(Maxis::RA, RegState::Undef);

  // Retain any imp-use flags.
  for (auto & MO : I->operands()) {
    if (MO.isImplicit())
      MIB.add(MO);
  }
}

void MaxisSEInstrInfo::expandERet(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator I) const {
  BuildMI(MBB, I, I->getDebugLoc(), get(Maxis::ERET));
}

std::pair<bool, bool>
MaxisSEInstrInfo::compareOpndSize(unsigned Opc,
                                 const MachineFunction &MF) const {
  const MCInstrDesc &Desc = get(Opc);
  assert(Desc.NumOperands == 2 && "Unary instruction expected.");
  const MaxisRegisterInfo *RI = &getRegisterInfo();
  unsigned DstRegSize = RI->getRegSizeInBits(*getRegClass(Desc, 0, RI, MF));
  unsigned SrcRegSize = RI->getRegSizeInBits(*getRegClass(Desc, 1, RI, MF));

  return std::make_pair(DstRegSize > SrcRegSize, DstRegSize < SrcRegSize);
}

void MaxisSEInstrInfo::expandPseudoMFHiLo(MachineBasicBlock &MBB,
                                         MachineBasicBlock::iterator I,
                                         unsigned NewOpc) const {
  BuildMI(MBB, I, I->getDebugLoc(), get(NewOpc), I->getOperand(0).getReg());
}

void MaxisSEInstrInfo::expandPseudoMTLoHi(MachineBasicBlock &MBB,
                                         MachineBasicBlock::iterator I,
                                         unsigned LoOpc,
                                         unsigned HiOpc,
                                         bool HasExplicitDef) const {
  // Expand
  //  lo_hi pseudomtlohi $gpr0, $gpr1
  // to these two instructions:
  //  mtlo $gpr0
  //  mthi $gpr1

  DebugLoc DL = I->getDebugLoc();
  const MachineOperand &SrcLo = I->getOperand(1), &SrcHi = I->getOperand(2);
  MachineInstrBuilder LoInst = BuildMI(MBB, I, DL, get(LoOpc));
  MachineInstrBuilder HiInst = BuildMI(MBB, I, DL, get(HiOpc));

  // Add lo/hi registers if the mtlo/hi instructions created have explicit
  // def registers.
  if (HasExplicitDef) {
    unsigned DstReg = I->getOperand(0).getReg();
    unsigned DstLo = getRegisterInfo().getSubReg(DstReg, Maxis::sub_lo);
    unsigned DstHi = getRegisterInfo().getSubReg(DstReg, Maxis::sub_hi);
    LoInst.addReg(DstLo, RegState::Define);
    HiInst.addReg(DstHi, RegState::Define);
  }

  LoInst.addReg(SrcLo.getReg(), getKillRegState(SrcLo.isKill()));
  HiInst.addReg(SrcHi.getReg(), getKillRegState(SrcHi.isKill()));
}

void MaxisSEInstrInfo::expandCvtFPInt(MachineBasicBlock &MBB,
                                     MachineBasicBlock::iterator I,
                                     unsigned CvtOpc, unsigned MovOpc,
                                     bool IsI64) const {
  const MCInstrDesc &CvtDesc = get(CvtOpc), &MovDesc = get(MovOpc);
  const MachineOperand &Dst = I->getOperand(0), &Src = I->getOperand(1);
  unsigned DstReg = Dst.getReg(), SrcReg = Src.getReg(), TmpReg = DstReg;
  unsigned KillSrc =  getKillRegState(Src.isKill());
  DebugLoc DL = I->getDebugLoc();
  bool DstIsLarger, SrcIsLarger;

  std::tie(DstIsLarger, SrcIsLarger) =
      compareOpndSize(CvtOpc, *MBB.getParent());

  if (DstIsLarger)
    TmpReg = getRegisterInfo().getSubReg(DstReg, Maxis::sub_lo);

  if (SrcIsLarger)
    DstReg = getRegisterInfo().getSubReg(DstReg, Maxis::sub_lo);

  BuildMI(MBB, I, DL, MovDesc, TmpReg).addReg(SrcReg, KillSrc);
  BuildMI(MBB, I, DL, CvtDesc, DstReg).addReg(TmpReg, RegState::Kill);
}

void MaxisSEInstrInfo::expandExtractElementF64(MachineBasicBlock &MBB,
                                              MachineBasicBlock::iterator I,
                                              bool FP64) const {
  unsigned DstReg = I->getOperand(0).getReg();
  unsigned SrcReg = I->getOperand(1).getReg();
  unsigned N = I->getOperand(2).getImm();
  DebugLoc dl = I->getDebugLoc();

  assert(N < 2 && "Invalid immediate");
  unsigned SubIdx = N ? Maxis::sub_hi : Maxis::sub_lo;
  unsigned SubReg = getRegisterInfo().getSubReg(SrcReg, SubIdx);

  // FPXX on MAXIS-II or MAXIS32r1 should have been handled with a spill/reload
  // in MaxisSEFrameLowering.cpp.
  assert(!(Subtarget.isABI_FPXX() && !Subtarget.hasMaxis32r2()));

  // FP64A (FP64 with nooddspreg) should have been handled with a spill/reload
  // in MaxisSEFrameLowering.cpp.
  assert(!(Subtarget.isFP64bit() && !Subtarget.useOddSPReg()));

  if (SubIdx == Maxis::sub_hi && Subtarget.hasMTHC1()) {
    // FIXME: Strictly speaking MFHC1 only reads the top 32-bits however, we
    //        claim to read the whole 64-bits as part of a white lie used to
    //        temporarily work around a widespread bug in the -mfp64 support.
    //        The problem is that none of the 32-bit fpu ops mention the fact
    //        that they clobber the upper 32-bits of the 64-bit FPR. Fixing that
    //        requires a major overhaul of the FPU implementation which can't
    //        be done right now due to time constraints.
    //        MFHC1 is one of two instructions that are affected since they are
    //        the only instructions that don't read the lower 32-bits.
    //        We therefore pretend that it reads the bottom 32-bits to
    //        artificially create a dependency and prevent the scheduler
    //        changing the behaviour of the code.
    BuildMI(MBB, I, dl, get(FP64 ? Maxis::MFHC1_D64 : Maxis::MFHC1_D32), DstReg)
        .addReg(SrcReg);
  } else
    BuildMI(MBB, I, dl, get(Maxis::MFC1), DstReg).addReg(SubReg);
}

void MaxisSEInstrInfo::expandBuildPairF64(MachineBasicBlock &MBB,
                                         MachineBasicBlock::iterator I,
                                         bool FP64) const {
  unsigned DstReg = I->getOperand(0).getReg();
  unsigned LoReg = I->getOperand(1).getReg(), HiReg = I->getOperand(2).getReg();
  const MCInstrDesc& Mtc1Tdd = get(Maxis::MTC1);
  DebugLoc dl = I->getDebugLoc();
  const TargetRegisterInfo &TRI = getRegisterInfo();

  // When mthc1 is available, use:
  //   mtc1 Lo, $fp
  //   mthc1 Hi, $fp
  //
  // Otherwise, for O32 FPXX ABI:
  //   spill + reload via ldc1
  // This case is handled by the frame lowering code.
  //
  // Otherwise, for FP32:
  //   mtc1 Lo, $fp
  //   mtc1 Hi, $fp + 1
  //
  // The case where dmtc1 is available doesn't need to be handled here
  // because it never creates a BuildPairF64 node.

  // FPXX on MAXIS-II or MAXIS32r1 should have been handled with a spill/reload
  // in MaxisSEFrameLowering.cpp.
  assert(!(Subtarget.isABI_FPXX() && !Subtarget.hasMaxis32r2()));

  // FP64A (FP64 with nooddspreg) should have been handled with a spill/reload
  // in MaxisSEFrameLowering.cpp.
  assert(!(Subtarget.isFP64bit() && !Subtarget.useOddSPReg()));

  BuildMI(MBB, I, dl, Mtc1Tdd, TRI.getSubReg(DstReg, Maxis::sub_lo))
    .addReg(LoReg);

  if (Subtarget.hasMTHC1()) {
    // FIXME: The .addReg(DstReg) is a white lie used to temporarily work
    //        around a widespread bug in the -mfp64 support.
    //        The problem is that none of the 32-bit fpu ops mention the fact
    //        that they clobber the upper 32-bits of the 64-bit FPR. Fixing that
    //        requires a major overhaul of the FPU implementation which can't
    //        be done right now due to time constraints.
    //        MTHC1 is one of two instructions that are affected since they are
    //        the only instructions that don't read the lower 32-bits.
    //        We therefore pretend that it reads the bottom 32-bits to
    //        artificially create a dependency and prevent the scheduler
    //        changing the behaviour of the code.
    BuildMI(MBB, I, dl, get(FP64 ? Maxis::MTHC1_D64 : Maxis::MTHC1_D32), DstReg)
        .addReg(DstReg)
        .addReg(HiReg);
  } else if (Subtarget.isABI_FPXX())
    llvm_unreachable("BuildPairF64 not expanded in frame lowering code!");
  else
    BuildMI(MBB, I, dl, Mtc1Tdd, TRI.getSubReg(DstReg, Maxis::sub_hi))
      .addReg(HiReg);
}

void MaxisSEInstrInfo::expandEhReturn(MachineBasicBlock &MBB,
                                     MachineBasicBlock::iterator I) const {
  // This pseudo instruction is generated as part of the lowering of
  // ISD::EH_RETURN. We convert it to a stack increment by OffsetReg, and
  // indirect jump to TargetReg
  MaxisABIInfo ABI = Subtarget.getABI();
  unsigned ADDU = ABI.GetPtrAdduOp();
  unsigned SP = Subtarget.isGP64bit() ? Maxis::SP_64 : Maxis::SP;
  unsigned RA = Subtarget.isGP64bit() ? Maxis::RA_64 : Maxis::RA;
  unsigned T9 = Subtarget.isGP64bit() ? Maxis::T9_64 : Maxis::T9;
  unsigned ZERO = Subtarget.isGP64bit() ? Maxis::ZERO_64 : Maxis::ZERO;
  unsigned OffsetReg = I->getOperand(0).getReg();
  unsigned TargetReg = I->getOperand(1).getReg();

  // addu $ra, $v0, $zero
  // addu $sp, $sp, $v1
  // jr   $ra (via RetRA)
  const TargetMachine &TM = MBB.getParent()->getTarget();
  if (TM.isPositionIndependent())
    BuildMI(MBB, I, I->getDebugLoc(), get(ADDU), T9)
        .addReg(TargetReg)
        .addReg(ZERO);
  BuildMI(MBB, I, I->getDebugLoc(), get(ADDU), RA)
      .addReg(TargetReg)
      .addReg(ZERO);
  BuildMI(MBB, I, I->getDebugLoc(), get(ADDU), SP).addReg(SP).addReg(OffsetReg);
  expandRetRA(MBB, I);
}

const MaxisInstrInfo *llvm::createMaxisSEInstrInfo(const MaxisSubtarget &STI) {
  return new MaxisSEInstrInfo(STI);
}
