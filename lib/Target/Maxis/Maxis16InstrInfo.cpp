//===- Maxis16InstrInfo.cpp - Maxis16 Instruction Information ---------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the Maxis16 implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "Maxis16InstrInfo.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/IR/DebugLoc.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/raw_ostream.h"
#include <cassert>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <vector>

using namespace llvm;

#define DEBUG_TYPE "maxis16-instrinfo"

Maxis16InstrInfo::Maxis16InstrInfo(const MaxisSubtarget &STI)
    : MaxisInstrInfo(STI, Maxis::Bimm16) {}

const MaxisRegisterInfo &Maxis16InstrInfo::getRegisterInfo() const {
  return RI;
}

/// isLoadFromStackSlot - If the specified machine instruction is a direct
/// load from a stack slot, return the virtual or physical register number of
/// the destination along with the FrameIndex of the loaded stack slot.  If
/// not, return 0.  This predicate must return 0 if the instruction has
/// any side effects other than loading from the stack slot.
unsigned Maxis16InstrInfo::isLoadFromStackSlot(const MachineInstr &MI,
                                              int &FrameIndex) const {
  return 0;
}

/// isStoreToStackSlot - If the specified machine instruction is a direct
/// store to a stack slot, return the virtual or physical register number of
/// the source reg along with the FrameIndex of the loaded stack slot.  If
/// not, return 0.  This predicate must return 0 if the instruction has
/// any side effects other than storing to the stack slot.
unsigned Maxis16InstrInfo::isStoreToStackSlot(const MachineInstr &MI,
                                             int &FrameIndex) const {
  return 0;
}

void Maxis16InstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                                  MachineBasicBlock::iterator I,
                                  const DebugLoc &DL, unsigned DestReg,
                                  unsigned SrcReg, bool KillSrc) const {
  unsigned Opc = 0;

  if (Maxis::CPU16RegsRegClass.contains(DestReg) &&
      Maxis::GPR32RegClass.contains(SrcReg))
    Opc = Maxis::MoveR3216;
  else if (Maxis::GPR32RegClass.contains(DestReg) &&
           Maxis::CPU16RegsRegClass.contains(SrcReg))
    Opc = Maxis::Move32R16;
  else if ((SrcReg == Maxis::HI0) &&
           (Maxis::CPU16RegsRegClass.contains(DestReg)))
    Opc = Maxis::Mfhi16, SrcReg = 0;
  else if ((SrcReg == Maxis::LO0) &&
           (Maxis::CPU16RegsRegClass.contains(DestReg)))
    Opc = Maxis::Mflo16, SrcReg = 0;

  assert(Opc && "Cannot copy registers");

  MachineInstrBuilder MIB = BuildMI(MBB, I, DL, get(Opc));

  if (DestReg)
    MIB.addReg(DestReg, RegState::Define);

  if (SrcReg)
    MIB.addReg(SrcReg, getKillRegState(KillSrc));
}

void Maxis16InstrInfo::storeRegToStack(MachineBasicBlock &MBB,
                                      MachineBasicBlock::iterator I,
                                      unsigned SrcReg, bool isKill, int FI,
                                      const TargetRegisterClass *RC,
                                      const TargetRegisterInfo *TRI,
                                      int64_t Offset) const {
  DebugLoc DL;
  if (I != MBB.end()) DL = I->getDebugLoc();
  MachineMemOperand *MMO = GetMemOperand(MBB, FI, MachineMemOperand::MOStore);
  unsigned Opc = 0;
  if (Maxis::CPU16RegsRegClass.hasSubClassEq(RC))
    Opc = Maxis::SwRxSpImmX16;
  assert(Opc && "Register class not handled!");
  BuildMI(MBB, I, DL, get(Opc)).addReg(SrcReg, getKillRegState(isKill)).
      addFrameIndex(FI).addImm(Offset)
      .addMemOperand(MMO);
}

void Maxis16InstrInfo::loadRegFromStack(MachineBasicBlock &MBB,
                                       MachineBasicBlock::iterator I,
                                       unsigned DestReg, int FI,
                                       const TargetRegisterClass *RC,
                                       const TargetRegisterInfo *TRI,
                                       int64_t Offset) const {
  DebugLoc DL;
  if (I != MBB.end()) DL = I->getDebugLoc();
  MachineMemOperand *MMO = GetMemOperand(MBB, FI, MachineMemOperand::MOLoad);
  unsigned Opc = 0;

  if (Maxis::CPU16RegsRegClass.hasSubClassEq(RC))
    Opc = Maxis::LwRxSpImmX16;
  assert(Opc && "Register class not handled!");
  BuildMI(MBB, I, DL, get(Opc), DestReg).addFrameIndex(FI).addImm(Offset)
    .addMemOperand(MMO);
}

bool Maxis16InstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
  MachineBasicBlock &MBB = *MI.getParent();
  switch (MI.getDesc().getOpcode()) {
  default:
    return false;
  case Maxis::RetRA16:
    ExpandRetRA16(MBB, MI, Maxis::JrcRa16);
    break;
  }

  MBB.erase(MI.getIterator());
  return true;
}

/// GetOppositeBranchOpc - Return the inverse of the specified
/// opcode, e.g. turning BEQ to BNE.
unsigned Maxis16InstrInfo::getOppositeBranchOpc(unsigned Opc) const {
  switch (Opc) {
  case Maxis::BeqzRxImmX16: return Maxis::BnezRxImmX16;
  case Maxis::BnezRxImmX16: return Maxis::BeqzRxImmX16;
  case Maxis::BeqzRxImm16: return Maxis::BnezRxImm16;
  case Maxis::BnezRxImm16: return Maxis::BeqzRxImm16;
  case Maxis::BteqzT8CmpX16: return Maxis::BtnezT8CmpX16;
  case Maxis::BteqzT8SltX16: return Maxis::BtnezT8SltX16;
  case Maxis::BteqzT8SltiX16: return Maxis::BtnezT8SltiX16;
  case Maxis::Btnez16: return Maxis::Bteqz16;
  case Maxis::BtnezX16: return Maxis::BteqzX16;
  case Maxis::BtnezT8CmpiX16: return Maxis::BteqzT8CmpiX16;
  case Maxis::BtnezT8SltuX16: return Maxis::BteqzT8SltuX16;
  case Maxis::BtnezT8SltiuX16: return Maxis::BteqzT8SltiuX16;
  case Maxis::Bteqz16: return Maxis::Btnez16;
  case Maxis::BteqzX16: return Maxis::BtnezX16;
  case Maxis::BteqzT8CmpiX16: return Maxis::BtnezT8CmpiX16;
  case Maxis::BteqzT8SltuX16: return Maxis::BtnezT8SltuX16;
  case Maxis::BteqzT8SltiuX16: return Maxis::BtnezT8SltiuX16;
  case Maxis::BtnezT8CmpX16: return Maxis::BteqzT8CmpX16;
  case Maxis::BtnezT8SltX16: return Maxis::BteqzT8SltX16;
  case Maxis::BtnezT8SltiX16: return Maxis::BteqzT8SltiX16;
  }
  llvm_unreachable("Illegal opcode!");
}

static void addSaveRestoreRegs(MachineInstrBuilder &MIB,
                               const std::vector<CalleeSavedInfo> &CSI,
                               unsigned Flags = 0) {
  for (unsigned i = 0, e = CSI.size(); i != e; ++i) {
    // Add the callee-saved register as live-in. Do not add if the register is
    // RA and return address is taken, because it has already been added in
    // method MaxisTargetLowering::lowerRETURNADDR.
    // It's killed at the spill, unless the register is RA and return address
    // is taken.
    unsigned Reg = CSI[e-i-1].getReg();
    switch (Reg) {
    case Maxis::RA:
    case Maxis::S0:
    case Maxis::S1:
      MIB.addReg(Reg, Flags);
      break;
    case Maxis::S2:
      break;
    default:
      llvm_unreachable("unexpected maxis16 callee saved register");

    }
  }
}

// Adjust SP by FrameSize bytes. Save RA, S0, S1
void Maxis16InstrInfo::makeFrame(unsigned SP, int64_t FrameSize,
                                MachineBasicBlock &MBB,
                                MachineBasicBlock::iterator I) const {
  DebugLoc DL;
  MachineFunction &MF = *MBB.getParent();
  MachineFrameInfo &MFI    = MF.getFrameInfo();
  const BitVector Reserved = RI.getReservedRegs(MF);
  bool SaveS2 = Reserved[Maxis::S2];
  MachineInstrBuilder MIB;
  unsigned Opc = ((FrameSize <= 128) && !SaveS2)? Maxis::Save16:Maxis::SaveX16;
  MIB = BuildMI(MBB, I, DL, get(Opc));
  const std::vector<CalleeSavedInfo> &CSI = MFI.getCalleeSavedInfo();
  addSaveRestoreRegs(MIB, CSI);
  if (SaveS2)
    MIB.addReg(Maxis::S2);
  if (isUInt<11>(FrameSize))
    MIB.addImm(FrameSize);
  else {
    int Base = 2040; // should create template function like isUInt that
                     // returns largest possible n bit unsigned integer
    int64_t Remainder = FrameSize - Base;
    MIB.addImm(Base);
    if (isInt<16>(-Remainder))
      BuildAddiSpImm(MBB, I, -Remainder);
    else
      adjustStackPtrBig(SP, -Remainder, MBB, I, Maxis::V0, Maxis::V1);
  }
}

// Adjust SP by FrameSize bytes. Restore RA, S0, S1
void Maxis16InstrInfo::restoreFrame(unsigned SP, int64_t FrameSize,
                                   MachineBasicBlock &MBB,
                                   MachineBasicBlock::iterator I) const {
  DebugLoc DL = I != MBB.end() ? I->getDebugLoc() : DebugLoc();
  MachineFunction *MF = MBB.getParent();
  MachineFrameInfo &MFI    = MF->getFrameInfo();
  const BitVector Reserved = RI.getReservedRegs(*MF);
  bool SaveS2 = Reserved[Maxis::S2];
  MachineInstrBuilder MIB;
  unsigned Opc = ((FrameSize <= 128) && !SaveS2)?
    Maxis::Restore16:Maxis::RestoreX16;

  if (!isUInt<11>(FrameSize)) {
    unsigned Base = 2040;
    int64_t Remainder = FrameSize - Base;
    FrameSize = Base; // should create template function like isUInt that
                     // returns largest possible n bit unsigned integer

    if (isInt<16>(Remainder))
      BuildAddiSpImm(MBB, I, Remainder);
    else
      adjustStackPtrBig(SP, Remainder, MBB, I, Maxis::A0, Maxis::A1);
  }
  MIB = BuildMI(MBB, I, DL, get(Opc));
  const std::vector<CalleeSavedInfo> &CSI = MFI.getCalleeSavedInfo();
  addSaveRestoreRegs(MIB, CSI, RegState::Define);
  if (SaveS2)
    MIB.addReg(Maxis::S2, RegState::Define);
  MIB.addImm(FrameSize);
}

// Adjust SP by Amount bytes where bytes can be up to 32bit number.
// This can only be called at times that we know that there is at least one free
// register.
// This is clearly safe at prologue and epilogue.
void Maxis16InstrInfo::adjustStackPtrBig(unsigned SP, int64_t Amount,
                                        MachineBasicBlock &MBB,
                                        MachineBasicBlock::iterator I,
                                        unsigned Reg1, unsigned Reg2) const {
  DebugLoc DL;
  //
  // li reg1, constant
  // move reg2, sp
  // add reg1, reg1, reg2
  // move sp, reg1
  //
  //
  MachineInstrBuilder MIB1 = BuildMI(MBB, I, DL, get(Maxis::LwConstant32), Reg1);
  MIB1.addImm(Amount).addImm(-1);
  MachineInstrBuilder MIB2 = BuildMI(MBB, I, DL, get(Maxis::MoveR3216), Reg2);
  MIB2.addReg(Maxis::SP, RegState::Kill);
  MachineInstrBuilder MIB3 = BuildMI(MBB, I, DL, get(Maxis::AdduRxRyRz16), Reg1);
  MIB3.addReg(Reg1);
  MIB3.addReg(Reg2, RegState::Kill);
  MachineInstrBuilder MIB4 = BuildMI(MBB, I, DL, get(Maxis::Move32R16),
                                                     Maxis::SP);
  MIB4.addReg(Reg1, RegState::Kill);
}

void Maxis16InstrInfo::adjustStackPtrBigUnrestricted(
    unsigned SP, int64_t Amount, MachineBasicBlock &MBB,
    MachineBasicBlock::iterator I) const {
   llvm_unreachable("adjust stack pointer amount exceeded");
}

/// Adjust SP by Amount bytes.
void Maxis16InstrInfo::adjustStackPtr(unsigned SP, int64_t Amount,
                                     MachineBasicBlock &MBB,
                                     MachineBasicBlock::iterator I) const {
  if (Amount == 0)
    return;

  if (isInt<16>(Amount))  // need to change to addi sp, ....and isInt<16>
    BuildAddiSpImm(MBB, I, Amount);
  else
    adjustStackPtrBigUnrestricted(SP, Amount, MBB, I);
}

/// This function generates the sequence of instructions needed to get the
/// result of adding register REG and immediate IMM.
unsigned Maxis16InstrInfo::loadImmediate(unsigned FrameReg, int64_t Imm,
                                        MachineBasicBlock &MBB,
                                        MachineBasicBlock::iterator II,
                                        const DebugLoc &DL,
                                        unsigned &NewImm) const {
  //
  // given original instruction is:
  // Instr rx, T[offset] where offset is too big.
  //
  // lo = offset & 0xFFFF
  // hi = ((offset >> 16) + (lo >> 15)) & 0xFFFF;
  //
  // let T = temporary register
  // li T, hi
  // shl T, 16
  // add T, Rx, T
  //
  RegScavenger rs;
  int32_t lo = Imm & 0xFFFF;
  NewImm = lo;
  int Reg =0;
  int SpReg = 0;

  rs.enterBasicBlock(MBB);
  rs.forward(II);
  //
  // We need to know which registers can be used, in the case where there
  // are not enough free registers. We exclude all registers that
  // are used in the instruction that we are helping.
  //  // Consider all allocatable registers in the register class initially
  BitVector Candidates =
      RI.getAllocatableSet
      (*II->getParent()->getParent(), &Maxis::CPU16RegsRegClass);
  // Exclude all the registers being used by the instruction.
  for (unsigned i = 0, e = II->getNumOperands(); i != e; ++i) {
    MachineOperand &MO = II->getOperand(i);
    if (MO.isReg() && MO.getReg() != 0 && !MO.isDef() &&
        !TargetRegisterInfo::isVirtualRegister(MO.getReg()))
      Candidates.reset(MO.getReg());
  }

  // If the same register was used and defined in an instruction, then
  // it will not be in the list of candidates.
  //
  // we need to analyze the instruction that we are helping.
  // we need to know if it defines register x but register x is not
  // present as an operand of the instruction. this tells
  // whether the register is live before the instruction. if it's not
  // then we don't need to save it in case there are no free registers.
  int DefReg = 0;
  for (unsigned i = 0, e = II->getNumOperands(); i != e; ++i) {
    MachineOperand &MO = II->getOperand(i);
    if (MO.isReg() && MO.isDef()) {
      DefReg = MO.getReg();
      break;
    }
  }

  BitVector Available = rs.getRegsAvailable(&Maxis::CPU16RegsRegClass);
  Available &= Candidates;
  //
  // we use T0 for the first register, if we need to save something away.
  // we use T1 for the second register, if we need to save something away.
  //
  unsigned FirstRegSaved =0, SecondRegSaved=0;
  unsigned FirstRegSavedTo = 0, SecondRegSavedTo = 0;

  Reg = Available.find_first();

  if (Reg == -1) {
    Reg = Candidates.find_first();
    Candidates.reset(Reg);
    if (DefReg != Reg) {
      FirstRegSaved = Reg;
      FirstRegSavedTo = Maxis::T0;
      copyPhysReg(MBB, II, DL, FirstRegSavedTo, FirstRegSaved, true);
    }
  }
  else
    Available.reset(Reg);
  BuildMI(MBB, II, DL, get(Maxis::LwConstant32), Reg).addImm(Imm).addImm(-1);
  NewImm = 0;
  if (FrameReg == Maxis::SP) {
    SpReg = Available.find_first();
    if (SpReg == -1) {
      SpReg = Candidates.find_first();
      // Candidates.reset(SpReg); // not really needed
      if (DefReg!= SpReg) {
        SecondRegSaved = SpReg;
        SecondRegSavedTo = Maxis::T1;
      }
      if (SecondRegSaved)
        copyPhysReg(MBB, II, DL, SecondRegSavedTo, SecondRegSaved, true);
    }
   else
     Available.reset(SpReg);
    copyPhysReg(MBB, II, DL, SpReg, Maxis::SP, false);
    BuildMI(MBB, II, DL, get(Maxis::  AdduRxRyRz16), Reg).addReg(SpReg, RegState::Kill)
      .addReg(Reg);
  }
  else
    BuildMI(MBB, II, DL, get(Maxis::  AdduRxRyRz16), Reg).addReg(FrameReg)
      .addReg(Reg, RegState::Kill);
  if (FirstRegSaved || SecondRegSaved) {
    II = std::next(II);
    if (FirstRegSaved)
      copyPhysReg(MBB, II, DL, FirstRegSaved, FirstRegSavedTo, true);
    if (SecondRegSaved)
      copyPhysReg(MBB, II, DL, SecondRegSaved, SecondRegSavedTo, true);
  }
  return Reg;
}

unsigned Maxis16InstrInfo::getAnalyzableBrOpc(unsigned Opc) const {
  return (Opc == Maxis::BeqzRxImmX16   || Opc == Maxis::BimmX16  ||
          Opc == Maxis::Bimm16  ||
          Opc == Maxis::Bteqz16        || Opc == Maxis::Btnez16 ||
          Opc == Maxis::BeqzRxImm16    || Opc == Maxis::BnezRxImm16   ||
          Opc == Maxis::BnezRxImmX16   || Opc == Maxis::BteqzX16 ||
          Opc == Maxis::BteqzT8CmpX16  || Opc == Maxis::BteqzT8CmpiX16 ||
          Opc == Maxis::BteqzT8SltX16  || Opc == Maxis::BteqzT8SltuX16  ||
          Opc == Maxis::BteqzT8SltiX16 || Opc == Maxis::BteqzT8SltiuX16 ||
          Opc == Maxis::BtnezX16       || Opc == Maxis::BtnezT8CmpX16 ||
          Opc == Maxis::BtnezT8CmpiX16 || Opc == Maxis::BtnezT8SltX16 ||
          Opc == Maxis::BtnezT8SltuX16 || Opc == Maxis::BtnezT8SltiX16 ||
          Opc == Maxis::BtnezT8SltiuX16 ) ? Opc : 0;
}

void Maxis16InstrInfo::ExpandRetRA16(MachineBasicBlock &MBB,
                                  MachineBasicBlock::iterator I,
                                  unsigned Opc) const {
  BuildMI(MBB, I, I->getDebugLoc(), get(Opc));
}

const MCInstrDesc &Maxis16InstrInfo::AddiSpImm(int64_t Imm) const {
  if (validSpImm8(Imm))
    return get(Maxis::AddiSpImm16);
  else
    return get(Maxis::AddiSpImmX16);
}

void Maxis16InstrInfo::BuildAddiSpImm
  (MachineBasicBlock &MBB, MachineBasicBlock::iterator I, int64_t Imm) const {
  DebugLoc DL;
  BuildMI(MBB, I, DL, AddiSpImm(Imm)).addImm(Imm);
}

const MaxisInstrInfo *llvm::createMaxis16InstrInfo(const MaxisSubtarget &STI) {
  return new Maxis16InstrInfo(STI);
}

bool Maxis16InstrInfo::validImmediate(unsigned Opcode, unsigned Reg,
                                     int64_t Amount) {
  switch (Opcode) {
  case Maxis::LbRxRyOffMemX16:
  case Maxis::LbuRxRyOffMemX16:
  case Maxis::LhRxRyOffMemX16:
  case Maxis::LhuRxRyOffMemX16:
  case Maxis::SbRxRyOffMemX16:
  case Maxis::ShRxRyOffMemX16:
  case Maxis::LwRxRyOffMemX16:
  case Maxis::SwRxRyOffMemX16:
  case Maxis::SwRxSpImmX16:
  case Maxis::LwRxSpImmX16:
    return isInt<16>(Amount);
  case Maxis::AddiRxRyOffMemX16:
    if ((Reg == Maxis::PC) || (Reg == Maxis::SP))
      return isInt<16>(Amount);
    return isInt<15>(Amount);
  }
  llvm_unreachable("unexpected Opcode in validImmediate");
}
