//=== MicroMaxisSizeReduction.cpp - MicroMaxis size reduction pass --------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
///\file
/// This pass is used to reduce the size of instructions where applicable.
///
/// TODO: Implement microMAXIS64 support.
/// TODO: Implement support for reducing into lwp/swp instruction.
//===----------------------------------------------------------------------===//
#include "Maxis.h"
#include "MaxisInstrInfo.h"
#include "MaxisSubtarget.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/Support/Debug.h"

using namespace llvm;

#define DEBUG_TYPE "micromaxis-reduce-size"

STATISTIC(NumReduced, "Number of 32-bit instructions reduced to 16-bit ones");

namespace {

/// Order of operands to transfer
// TODO: Will be extended when additional optimizations are added
enum OperandTransfer {
  OT_NA,          ///< Not applicable
  OT_OperandsAll, ///< Transfer all operands
  OT_Operands02,  ///< Transfer operands 0 and 2
  OT_Operand2,    ///< Transfer just operand 2
  OT_OperandsXOR, ///< Transfer operands for XOR16
};

/// Reduction type
// TODO: Will be extended when additional optimizations are added
enum ReduceType {
  RT_OneInstr ///< Reduce one instruction into a smaller instruction
};

// Information about immediate field restrictions
struct ImmField {
  ImmField() : ImmFieldOperand(-1), Shift(0), LBound(0), HBound(0) {}
  ImmField(uint8_t Shift, int16_t LBound, int16_t HBound,
           int8_t ImmFieldOperand)
      : ImmFieldOperand(ImmFieldOperand), Shift(Shift), LBound(LBound),
        HBound(HBound) {}
  int8_t ImmFieldOperand; // Immediate operand, -1 if it does not exist
  uint8_t Shift;          // Shift value
  int16_t LBound;         // Low bound of the immediate operand
  int16_t HBound;         // High bound of the immediate operand
};

/// Information about operands
// TODO: Will be extended when additional optimizations are added
struct OpInfo {
  OpInfo(enum OperandTransfer TransferOperands)
      : TransferOperands(TransferOperands) {}
  OpInfo() : TransferOperands(OT_NA) {}

  enum OperandTransfer
      TransferOperands; ///< Operands to transfer to the new instruction
};

// Information about opcodes
struct OpCodes {
  OpCodes(unsigned WideOpc, unsigned NarrowOpc)
      : WideOpc(WideOpc), NarrowOpc(NarrowOpc) {}

  unsigned WideOpc;   ///< Wide opcode
  unsigned NarrowOpc; ///< Narrow opcode
};

/// ReduceTable - A static table with information on mapping from wide
/// opcodes to narrow
struct ReduceEntry {

  enum ReduceType eRType; ///< Reduction type
  bool (*ReduceFunction)(
      MachineInstr *MI,
      const ReduceEntry &Entry); ///< Pointer to reduce function
  struct OpCodes Ops;            ///< All relevant OpCodes
  struct OpInfo OpInf;           ///< Characteristics of operands
  struct ImmField Imm;           ///< Characteristics of immediate field

  ReduceEntry(enum ReduceType RType, struct OpCodes Op,
              bool (*F)(MachineInstr *MI, const ReduceEntry &Entry),
              struct OpInfo OpInf, struct ImmField Imm)
      : eRType(RType), ReduceFunction(F), Ops(Op), OpInf(OpInf), Imm(Imm) {}

  unsigned NarrowOpc() const { return Ops.NarrowOpc; }
  unsigned WideOpc() const { return Ops.WideOpc; }
  int16_t LBound() const { return Imm.LBound; }
  int16_t HBound() const { return Imm.HBound; }
  uint8_t Shift() const { return Imm.Shift; }
  int8_t ImmField() const { return Imm.ImmFieldOperand; }
  enum OperandTransfer TransferOperands() const {
    return OpInf.TransferOperands;
  }
  enum ReduceType RType() const { return eRType; }

  // operator used by std::equal_range
  bool operator<(const unsigned int r) const { return (WideOpc() < r); }

  // operator used by std::equal_range
  friend bool operator<(const unsigned int r, const struct ReduceEntry &re) {
    return (r < re.WideOpc());
  }
};

class MicroMaxisSizeReduce : public MachineFunctionPass {
public:
  static char ID;
  MicroMaxisSizeReduce();

  static const MaxisInstrInfo *MaxisII;
  const MaxisSubtarget *Subtarget;

  bool runOnMachineFunction(MachineFunction &MF) override;

  llvm::StringRef getPassName() const override {
    return "microMAXIS instruction size reduction pass";
  }

private:
  /// Reduces width of instructions in the specified basic block.
  bool ReduceMBB(MachineBasicBlock &MBB);

  /// Attempts to reduce MI, returns true on success.
  bool ReduceMI(const MachineBasicBlock::instr_iterator &MII);

  // Attempts to reduce LW/SW instruction into LWSP/SWSP,
  // returns true on success.
  static bool ReduceXWtoXWSP(MachineInstr *MI, const ReduceEntry &Entry);

  // Attempts to reduce LBU/LHU instruction into LBU16/LHU16,
  // returns true on success.
  static bool ReduceLXUtoLXU16(MachineInstr *MI, const ReduceEntry &Entry);

  // Attempts to reduce SB/SH instruction into SB16/SH16,
  // returns true on success.
  static bool ReduceSXtoSX16(MachineInstr *MI, const ReduceEntry &Entry);

  // Attempts to reduce arithmetic instructions, returns true on success.
  static bool ReduceArithmeticInstructions(MachineInstr *MI,
                                           const ReduceEntry &Entry);

  // Attempts to reduce ADDI into ADDISP instruction,
  // returns true on success.
  static bool ReduceADDIToADDISP(MachineInstr *MI, const ReduceEntry &Entry);

  // Attempts to reduce ADDI into ADDIR1SP instruction,
  // returns true on success.
  static bool ReduceADDIToADDIR1SP(MachineInstr *MI,
                                     const ReduceEntry &Entry);

  // Attempts to reduce XOR into XOR16 instruction,
  // returns true on success.
  static bool ReduceXORtoXOR16(MachineInstr *MI, const ReduceEntry &Entry);

  // Changes opcode of an instruction.
  static bool ReplaceInstruction(MachineInstr *MI, const ReduceEntry &Entry);

  // Table with transformation rules for each instruction.
  static llvm::SmallVector<ReduceEntry, 16> ReduceTable;
};

char MicroMaxisSizeReduce::ID = 0;
const MaxisInstrInfo *MicroMaxisSizeReduce::MaxisII;

// This table must be sorted by WideOpc as a main criterion and
// ReduceType as a sub-criterion (when wide opcodes are the same).
llvm::SmallVector<ReduceEntry, 16> MicroMaxisSizeReduce::ReduceTable = {

    // ReduceType, OpCodes, ReduceFunction,
    // OpInfo(TransferOperands),
    // ImmField(Shift, LBound, HBound, ImmFieldPosition)
    {RT_OneInstr, OpCodes(Maxis::ADDi, Maxis::ADDIR1SP_MM),
     ReduceADDIToADDIR1SP, OpInfo(OT_Operands02), ImmField(2, 0, 64, 2)},
    {RT_OneInstr, OpCodes(Maxis::ADDi, Maxis::ADDISP_MM), ReduceADDIToADDISP,
     OpInfo(OT_Operand2), ImmField(0, 0, 0, 2)},
    {RT_OneInstr, OpCodes(Maxis::ADDi_MM, Maxis::ADDIR1SP_MM),
     ReduceADDIToADDIR1SP, OpInfo(OT_Operands02), ImmField(2, 0, 64, 2)},
    {RT_OneInstr, OpCodes(Maxis::ADDi_MM, Maxis::ADDISP_MM),
     ReduceADDIToADDISP, OpInfo(OT_Operand2), ImmField(0, 0, 0, 2)},
    {RT_OneInstr, OpCodes(Maxis::ADDu, Maxis::ADDU16_MM),
     ReduceArithmeticInstructions, OpInfo(OT_OperandsAll),
     ImmField(0, 0, 0, -1)},
    {RT_OneInstr, OpCodes(Maxis::ADDu_MM, Maxis::ADDU16_MM),
     ReduceArithmeticInstructions, OpInfo(OT_OperandsAll),
     ImmField(0, 0, 0, -1)},
    {RT_OneInstr, OpCodes(Maxis::LBu, Maxis::LBU16_MM), ReduceLXUtoLXU16,
     OpInfo(OT_OperandsAll), ImmField(0, -1, 15, 2)},
    {RT_OneInstr, OpCodes(Maxis::LBu_MM, Maxis::LBU16_MM), ReduceLXUtoLXU16,
     OpInfo(OT_OperandsAll), ImmField(0, -1, 15, 2)},
    {RT_OneInstr, OpCodes(Maxis::LEA_ADDi, Maxis::ADDIR1SP_MM),
     ReduceADDIToADDIR1SP, OpInfo(OT_Operands02), ImmField(2, 0, 64, 2)},
    {RT_OneInstr, OpCodes(Maxis::LHu, Maxis::LHU16_MM), ReduceLXUtoLXU16,
     OpInfo(OT_OperandsAll), ImmField(1, 0, 16, 2)},
    {RT_OneInstr, OpCodes(Maxis::LHu_MM, Maxis::LHU16_MM), ReduceLXUtoLXU16,
     OpInfo(OT_OperandsAll), ImmField(1, 0, 16, 2)},
    {RT_OneInstr, OpCodes(Maxis::LW, Maxis::LWSP_MM), ReduceXWtoXWSP,
     OpInfo(OT_OperandsAll), ImmField(2, 0, 32, 2)},
    {RT_OneInstr, OpCodes(Maxis::LW_MM, Maxis::LWSP_MM), ReduceXWtoXWSP,
     OpInfo(OT_OperandsAll), ImmField(2, 0, 32, 2)},
    {RT_OneInstr, OpCodes(Maxis::SB, Maxis::SB16_MM), ReduceSXtoSX16,
     OpInfo(OT_OperandsAll), ImmField(0, 0, 16, 2)},
    {RT_OneInstr, OpCodes(Maxis::SB_MM, Maxis::SB16_MM), ReduceSXtoSX16,
     OpInfo(OT_OperandsAll), ImmField(0, 0, 16, 2)},
    {RT_OneInstr, OpCodes(Maxis::SH, Maxis::SH16_MM), ReduceSXtoSX16,
     OpInfo(OT_OperandsAll), ImmField(1, 0, 16, 2)},
    {RT_OneInstr, OpCodes(Maxis::SH_MM, Maxis::SH16_MM), ReduceSXtoSX16,
     OpInfo(OT_OperandsAll), ImmField(1, 0, 16, 2)},
    {RT_OneInstr, OpCodes(Maxis::SUBu, Maxis::SUBU16_MM),
     ReduceArithmeticInstructions, OpInfo(OT_OperandsAll),
     ImmField(0, 0, 0, -1)},
    {RT_OneInstr, OpCodes(Maxis::SUBu_MM, Maxis::SUBU16_MM),
     ReduceArithmeticInstructions, OpInfo(OT_OperandsAll),
     ImmField(0, 0, 0, -1)},
    {RT_OneInstr, OpCodes(Maxis::SW, Maxis::SWSP_MM), ReduceXWtoXWSP,
     OpInfo(OT_OperandsAll), ImmField(2, 0, 32, 2)},
    {RT_OneInstr, OpCodes(Maxis::SW_MM, Maxis::SWSP_MM), ReduceXWtoXWSP,
     OpInfo(OT_OperandsAll), ImmField(2, 0, 32, 2)},
    {RT_OneInstr, OpCodes(Maxis::XOR, Maxis::XOR16_MM), ReduceXORtoXOR16,
     OpInfo(OT_OperandsXOR), ImmField(0, 0, 0, -1)},
    {RT_OneInstr, OpCodes(Maxis::XOR_MM, Maxis::XOR16_MM), ReduceXORtoXOR16,
     OpInfo(OT_OperandsXOR), ImmField(0, 0, 0, -1)}};
} // namespace

// Returns true if the machine operand MO is register SP.
static bool IsSP(const MachineOperand &MO) {
  if (MO.isReg() && ((MO.getReg() == Maxis::SP)))
    return true;
  return false;
}

// Returns true if the machine operand MO is register $16, $17, or $2-$7.
static bool isMMThreeBitGPRegister(const MachineOperand &MO) {
  if (MO.isReg() && Maxis::GPRMM16RegClass.contains(MO.getReg()))
    return true;
  return false;
}

// Returns true if the machine operand MO is register $0, $17, or $2-$7.
static bool isMMSourceRegister(const MachineOperand &MO) {
  if (MO.isReg() && Maxis::GPRMM16ZeroRegClass.contains(MO.getReg()))
    return true;
  return false;
}

// Returns true if the operand Op is an immediate value
// and writes the immediate value into variable Imm.
static bool GetImm(MachineInstr *MI, unsigned Op, int64_t &Imm) {

  if (!MI->getOperand(Op).isImm())
    return false;
  Imm = MI->getOperand(Op).getImm();
  return true;
}

// Returns true if the value is a valid immediate for ADDISP.
static bool AddispImmValue(int64_t Value) {
  int64_t Value2 = Value >> 2;
  if (((Value & (int64_t)maskTrailingZeros<uint64_t>(2)) == Value) &&
      ((Value2 >= 2 && Value2 <= 257) || (Value2 >= -258 && Value2 <= -3)))
    return true;
  return false;
}

// Returns true if the variable Value has the number of least-significant zero
// bits equal to Shift and if the shifted value is between the bounds.
static bool InRange(int64_t Value, unsigned short Shift, int LBound,
                    int HBound) {
  int64_t Value2 = Value >> Shift;
  if (((Value & (int64_t)maskTrailingZeros<uint64_t>(Shift)) == Value) &&
      (Value2 >= LBound) && (Value2 < HBound))
    return true;
  return false;
}

// Returns true if immediate operand is in range.
static bool ImmInRange(MachineInstr *MI, const ReduceEntry &Entry) {

  int64_t offset;

  if (!GetImm(MI, Entry.ImmField(), offset))
    return false;

  if (!InRange(offset, Entry.Shift(), Entry.LBound(), Entry.HBound()))
    return false;

  return true;
}

MicroMaxisSizeReduce::MicroMaxisSizeReduce() : MachineFunctionPass(ID) {}

bool MicroMaxisSizeReduce::ReduceMI(
    const MachineBasicBlock::instr_iterator &MII) {

  MachineInstr *MI = &*MII;
  unsigned Opcode = MI->getOpcode();

  // Search the table.
  llvm::SmallVector<ReduceEntry, 16>::const_iterator Start =
      std::begin(ReduceTable);
  llvm::SmallVector<ReduceEntry, 16>::const_iterator End =
      std::end(ReduceTable);

  std::pair<llvm::SmallVector<ReduceEntry, 16>::const_iterator,
            llvm::SmallVector<ReduceEntry, 16>::const_iterator>
      Range = std::equal_range(Start, End, Opcode);

  if (Range.first == Range.second)
    return false;

  for (llvm::SmallVector<ReduceEntry, 16>::const_iterator Entry = Range.first;
       Entry != Range.second; ++Entry)
    if (((*Entry).ReduceFunction)(&(*MII), *Entry))
      return true;

  return false;
}

bool MicroMaxisSizeReduce::ReduceXWtoXWSP(MachineInstr *MI,
                                         const ReduceEntry &Entry) {

  if (!ImmInRange(MI, Entry))
    return false;

  if (!IsSP(MI->getOperand(1)))
    return false;

  return ReplaceInstruction(MI, Entry);
}

bool MicroMaxisSizeReduce::ReduceArithmeticInstructions(
    MachineInstr *MI, const ReduceEntry &Entry) {

  if (!isMMThreeBitGPRegister(MI->getOperand(0)) ||
      !isMMThreeBitGPRegister(MI->getOperand(1)) ||
      !isMMThreeBitGPRegister(MI->getOperand(2)))
    return false;

  return ReplaceInstruction(MI, Entry);
}

bool MicroMaxisSizeReduce::ReduceADDIToADDIR1SP(MachineInstr *MI,
                                                 const ReduceEntry &Entry) {

  if (!ImmInRange(MI, Entry))
    return false;

  if (!isMMThreeBitGPRegister(MI->getOperand(0)) || !IsSP(MI->getOperand(1)))
    return false;

  return ReplaceInstruction(MI, Entry);
}

bool MicroMaxisSizeReduce::ReduceADDIToADDISP(MachineInstr *MI,
                                               const ReduceEntry &Entry) {

  int64_t ImmValue;
  if (!GetImm(MI, Entry.ImmField(), ImmValue))
    return false;

  if (!AddispImmValue(ImmValue))
    return false;

  if (!IsSP(MI->getOperand(0)) || !IsSP(MI->getOperand(1)))
    return false;

  return ReplaceInstruction(MI, Entry);
}

bool MicroMaxisSizeReduce::ReduceLXUtoLXU16(MachineInstr *MI,
                                           const ReduceEntry &Entry) {

  if (!ImmInRange(MI, Entry))
    return false;

  if (!isMMThreeBitGPRegister(MI->getOperand(0)) ||
      !isMMThreeBitGPRegister(MI->getOperand(1)))
    return false;

  return ReplaceInstruction(MI, Entry);
}

bool MicroMaxisSizeReduce::ReduceSXtoSX16(MachineInstr *MI,
                                         const ReduceEntry &Entry) {

  if (!ImmInRange(MI, Entry))
    return false;

  if (!isMMSourceRegister(MI->getOperand(0)) ||
      !isMMThreeBitGPRegister(MI->getOperand(1)))
    return false;

  return ReplaceInstruction(MI, Entry);
}

bool MicroMaxisSizeReduce::ReduceXORtoXOR16(MachineInstr *MI,
                                           const ReduceEntry &Entry) {
  if (!isMMThreeBitGPRegister(MI->getOperand(0)) ||
      !isMMThreeBitGPRegister(MI->getOperand(1)) ||
      !isMMThreeBitGPRegister(MI->getOperand(2)))
    return false;

  if (!(MI->getOperand(0).getReg() == MI->getOperand(2).getReg()) &&
      !(MI->getOperand(0).getReg() == MI->getOperand(1).getReg()))
    return false;

  return ReplaceInstruction(MI, Entry);
}

bool MicroMaxisSizeReduce::ReduceMBB(MachineBasicBlock &MBB) {
  bool Modified = false;
  MachineBasicBlock::instr_iterator MII = MBB.instr_begin(),
                                    E = MBB.instr_end();
  MachineBasicBlock::instr_iterator NextMII;

  // Iterate through the instructions in the basic block
  for (; MII != E; MII = NextMII) {
    NextMII = std::next(MII);
    MachineInstr *MI = &*MII;

    // Don't reduce bundled instructions or pseudo operations
    if (MI->isBundle() || MI->isTransient())
      continue;

    // Try to reduce 32-bit instruction into 16-bit instruction
    Modified |= ReduceMI(MII);
  }

  return Modified;
}

bool MicroMaxisSizeReduce::ReplaceInstruction(MachineInstr *MI,
                                             const ReduceEntry &Entry) {

  enum OperandTransfer OpTransfer = Entry.TransferOperands();

  DEBUG(dbgs() << "Converting 32-bit: " << *MI);
  ++NumReduced;

  if (OpTransfer == OT_OperandsAll) {
    MI->setDesc(MaxisII->get(Entry.NarrowOpc()));
    DEBUG(dbgs() << "       to 16-bit: " << *MI);
    return true;
  } else {
    MachineBasicBlock &MBB = *MI->getParent();
    const MCInstrDesc &NewMCID = MaxisII->get(Entry.NarrowOpc());
    DebugLoc dl = MI->getDebugLoc();
    MachineInstrBuilder MIB = BuildMI(MBB, MI, dl, NewMCID);
    switch (OpTransfer) {
    case OT_Operand2:
      MIB.add(MI->getOperand(2));
      break;
    case OT_Operands02: {
      MIB.add(MI->getOperand(0));
      MIB.add(MI->getOperand(2));
      break;
    }
    case OT_OperandsXOR: {
      if (MI->getOperand(0).getReg() == MI->getOperand(2).getReg()) {
        MIB.add(MI->getOperand(0));
        MIB.add(MI->getOperand(1));
        MIB.add(MI->getOperand(2));
      } else {
        MIB.add(MI->getOperand(0));
        MIB.add(MI->getOperand(2));
        MIB.add(MI->getOperand(1));
      }
      break;
    }
    default:
      llvm_unreachable("Unknown operand transfer!");
    }

    // Transfer MI flags.
    MIB.setMIFlags(MI->getFlags());

    DEBUG(dbgs() << "       to 16-bit: " << *MIB);
    MBB.erase_instr(MI);
    return true;
  }
  return false;
}

bool MicroMaxisSizeReduce::runOnMachineFunction(MachineFunction &MF) {

  Subtarget = &static_cast<const MaxisSubtarget &>(MF.getSubtarget());

  // TODO: Add support for the subtarget microMAXIS32R6.
  if (!Subtarget->inMicroMaxisMode() || !Subtarget->hasMaxis32r2() ||
      Subtarget->hasMaxis32r6())
    return false;

  MaxisII = static_cast<const MaxisInstrInfo *>(Subtarget->getInstrInfo());

  bool Modified = false;
  MachineFunction::iterator I = MF.begin(), E = MF.end();

  for (; I != E; ++I)
    Modified |= ReduceMBB(*I);
  return Modified;
}

/// Returns an instance of the MicroMaxis size reduction pass.
FunctionPass *llvm::createMicroMaxisSizeReductionPass() {
  return new MicroMaxisSizeReduce();
}
