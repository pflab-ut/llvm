//===- MaxisISelLowering.cpp - Maxis DAG Lowering Implementation ------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines the interfaces that Maxis uses to lower LLVM code into a
// selection DAG.
//
//===----------------------------------------------------------------------===//

#include "MaxisISelLowering.h"
#include "InstPrinter/MaxisInstPrinter.h"
#include "MCTargetDesc/MaxisBaseInfo.h"
#include "MCTargetDesc/MaxisMCTargetDesc.h"
#include "MaxisCCState.h"
#include "MaxisInstrInfo.h"
#include "MaxisMachineFunction.h"
#include "MaxisRegisterInfo.h"
#include "MaxisSubtarget.h"
#include "MaxisTargetMachine.h"
#include "MaxisTargetObjectFile.h"
#include "llvm/ADT/APFloat.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/FunctionLoweringInfo.h"
#include "llvm/CodeGen/ISDOpcodes.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineJumpTableInfo.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/MachineValueType.h"
#include "llvm/CodeGen/RuntimeLibcalls.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/CodeGen/SelectionDAGNodes.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/ValueTypes.h"
#include "llvm/IR/CallingConv.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/DebugLoc.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/GlobalValue.h"
#include "llvm/IR/Type.h"
#include "llvm/IR/Value.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/CodeGen.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetOptions.h"
#include <algorithm>
#include <cassert>
#include <cctype>
#include <cstdint>
#include <deque>
#include <iterator>
#include <utility>
#include <vector>

using namespace llvm;

#define DEBUG_TYPE "maxis-lower"

STATISTIC(NumTailCalls, "Number of tail calls");

static cl::opt<bool>
LargeGOT("mxgot", cl::Hidden,
         cl::desc("MAXIS: Enable GOT larger than 64k."), cl::init(false));

static cl::opt<bool>
NoZeroDivCheck("mno-check-zero-division", cl::Hidden,
               cl::desc("MAXIS: Don't trap on integer division by zero."),
               cl::init(false));

static const MCPhysReg Maxis64DPRegs[8] = {
  Maxis::D12_64, Maxis::D13_64, Maxis::D14_64, Maxis::D15_64,
  Maxis::D16_64, Maxis::D17_64, Maxis::D18_64, Maxis::D19_64
};

// If I is a shifted mask, set the size (Size) and the first bit of the
// mask (Pos), and return true.
// For example, if I is 0x003ff800, (Pos, Size) = (11, 11).
static bool isShiftedMask(uint64_t I, uint64_t &Pos, uint64_t &Size) {
  if (!isShiftedMask_64(I))
    return false;

  Size = countPopulation(I);
  Pos = countTrailingZeros(I);
  return true;
}

// The MAXIS MSA ABI passes vector arguments in the integer register set.
// The number of integer registers used is dependant on the ABI used.
MVT MaxisTargetLowering::getRegisterTypeForCallingConv(MVT VT) const {
  if (VT.isVector() && Subtarget.hasMSA())
    return Subtarget.isABI_O32() ? MVT::i32 : MVT::i64;
  return MaxisTargetLowering::getRegisterType(VT);
}

MVT MaxisTargetLowering::getRegisterTypeForCallingConv(LLVMContext &Context,
                                                      EVT VT) const {
  if (VT.isVector()) {
      if (Subtarget.isABI_O32()) {
        return MVT::i32;
      } else {
        return (VT.getSizeInBits() == 32) ? MVT::i32 : MVT::i64;
      }
  }
  return MaxisTargetLowering::getRegisterType(Context, VT);
}

unsigned MaxisTargetLowering::getNumRegistersForCallingConv(LLVMContext &Context,
                                                           EVT VT) const {
  if (VT.isVector())
    return std::max((VT.getSizeInBits() / (Subtarget.isABI_O32() ? 32 : 64)),
                    1U);
  return MaxisTargetLowering::getNumRegisters(Context, VT);
}

unsigned MaxisTargetLowering::getVectorTypeBreakdownForCallingConv(
    LLVMContext &Context, EVT VT, EVT &IntermediateVT,
    unsigned &NumIntermediates, MVT &RegisterVT) const {
  // Break down vector types to either 2 i64s or 4 i32s.
  RegisterVT = getRegisterTypeForCallingConv(Context, VT) ;
  IntermediateVT = RegisterVT;
  NumIntermediates = VT.getSizeInBits() < RegisterVT.getSizeInBits()
                         ? VT.getVectorNumElements()
                         : VT.getSizeInBits() / RegisterVT.getSizeInBits();

  return NumIntermediates;
}

SDValue MaxisTargetLowering::getGlobalReg(SelectionDAG &DAG, EVT Ty) const {
  MaxisFunctionInfo *FI = DAG.getMachineFunction().getInfo<MaxisFunctionInfo>();
  return DAG.getRegister(FI->getGlobalBaseReg(), Ty);
}

SDValue MaxisTargetLowering::getTargetNode(GlobalAddressSDNode *N, EVT Ty,
                                          SelectionDAG &DAG,
                                          unsigned Flag) const {
  return DAG.getTargetGlobalAddress(N->getGlobal(), SDLoc(N), Ty, 0, Flag);
}

SDValue MaxisTargetLowering::getTargetNode(ExternalSymbolSDNode *N, EVT Ty,
                                          SelectionDAG &DAG,
                                          unsigned Flag) const {
  return DAG.getTargetExternalSymbol(N->getSymbol(), Ty, Flag);
}

SDValue MaxisTargetLowering::getTargetNode(BlockAddressSDNode *N, EVT Ty,
                                          SelectionDAG &DAG,
                                          unsigned Flag) const {
  return DAG.getTargetBlockAddress(N->getBlockAddress(), Ty, 0, Flag);
}

SDValue MaxisTargetLowering::getTargetNode(JumpTableSDNode *N, EVT Ty,
                                          SelectionDAG &DAG,
                                          unsigned Flag) const {
  return DAG.getTargetJumpTable(N->getIndex(), Ty, Flag);
}

SDValue MaxisTargetLowering::getTargetNode(ConstantPoolSDNode *N, EVT Ty,
                                          SelectionDAG &DAG,
                                          unsigned Flag) const {
  return DAG.getTargetConstantPool(N->getConstVal(), Ty, N->getAlignment(),
                                   N->getOffset(), Flag);
}

const char *MaxisTargetLowering::getTargetNodeName(unsigned Opcode) const {
  switch ((MaxisISD::NodeType)Opcode) {
  case MaxisISD::FIRST_NUMBER:      break;
  case MaxisISD::JmpLink:           return "MaxisISD::JmpLink";
  case MaxisISD::TailCall:          return "MaxisISD::TailCall";
  case MaxisISD::Highest:           return "MaxisISD::Highest";
  case MaxisISD::Higher:            return "MaxisISD::Higher";
  case MaxisISD::Hi:                return "MaxisISD::Hi";
  case MaxisISD::Lo:                return "MaxisISD::Lo";
  case MaxisISD::GotHi:             return "MaxisISD::GotHi";
  case MaxisISD::GPRel:             return "MaxisISD::GPRel";
  case MaxisISD::ThreadPointer:     return "MaxisISD::ThreadPointer";
  case MaxisISD::Ret:               return "MaxisISD::Ret";
  case MaxisISD::ERet:              return "MaxisISD::ERet";
  case MaxisISD::EH_RETURN:         return "MaxisISD::EH_RETURN";
  case MaxisISD::FPBrcond:          return "MaxisISD::FPBrcond";
  case MaxisISD::FPCmp:             return "MaxisISD::FPCmp";
  case MaxisISD::FSELECT:           return "MaxisISD::FSELECT";
  case MaxisISD::MTC1_D64:          return "MaxisISD::MTC1_D64";
  case MaxisISD::CMovFP_T:          return "MaxisISD::CMovFP_T";
  case MaxisISD::CMovFP_F:          return "MaxisISD::CMovFP_F";
  case MaxisISD::TruncIntFP:        return "MaxisISD::TruncIntFP";
  case MaxisISD::MFHI:              return "MaxisISD::MFHI";
  case MaxisISD::MFLO:              return "MaxisISD::MFLO";
  case MaxisISD::MTLOHI:            return "MaxisISD::MTLOHI";
  case MaxisISD::Mult:              return "MaxisISD::Mult";
  case MaxisISD::Multu:             return "MaxisISD::Multu";
  case MaxisISD::MAdd:              return "MaxisISD::MAdd";
  case MaxisISD::MAddu:             return "MaxisISD::MAddu";
  case MaxisISD::MSub:              return "MaxisISD::MSub";
  case MaxisISD::MSubu:             return "MaxisISD::MSubu";
  case MaxisISD::DivRem:            return "MaxisISD::DivRem";
  case MaxisISD::DivRemU:           return "MaxisISD::DivRemU";
  case MaxisISD::DivRem16:          return "MaxisISD::DivRem16";
  case MaxisISD::DivRemU16:         return "MaxisISD::DivRemU16";
  case MaxisISD::BuildPairF64:      return "MaxisISD::BuildPairF64";
  case MaxisISD::ExtractElementF64: return "MaxisISD::ExtractElementF64";
  case MaxisISD::Wrapper:           return "MaxisISD::Wrapper";
  case MaxisISD::DynAlloc:          return "MaxisISD::DynAlloc";
  case MaxisISD::Sync:              return "MaxisISD::Sync";
  case MaxisISD::Ext:               return "MaxisISD::Ext";
  case MaxisISD::Ins:               return "MaxisISD::Ins";
  case MaxisISD::CIns:              return "MaxisISD::CIns";
  case MaxisISD::LWL:               return "MaxisISD::LWL";
  case MaxisISD::LWR:               return "MaxisISD::LWR";
  case MaxisISD::SWL:               return "MaxisISD::SWL";
  case MaxisISD::SWR:               return "MaxisISD::SWR";
  case MaxisISD::LDL:               return "MaxisISD::LDL";
  case MaxisISD::LDR:               return "MaxisISD::LDR";
  case MaxisISD::SDL:               return "MaxisISD::SDL";
  case MaxisISD::SDR:               return "MaxisISD::SDR";
  case MaxisISD::EXTP:              return "MaxisISD::EXTP";
  case MaxisISD::EXTPDP:            return "MaxisISD::EXTPDP";
  case MaxisISD::EXTR_S_H:          return "MaxisISD::EXTR_S_H";
  case MaxisISD::EXTR_W:            return "MaxisISD::EXTR_W";
  case MaxisISD::EXTR_R_W:          return "MaxisISD::EXTR_R_W";
  case MaxisISD::EXTR_RS_W:         return "MaxisISD::EXTR_RS_W";
  case MaxisISD::SHILO:             return "MaxisISD::SHILO";
  case MaxisISD::MTHLIP:            return "MaxisISD::MTHLIP";
  case MaxisISD::MULSAQ_S_W_PH:     return "MaxisISD::MULSAQ_S_W_PH";
  case MaxisISD::MAQ_S_W_PHL:       return "MaxisISD::MAQ_S_W_PHL";
  case MaxisISD::MAQ_S_W_PHR:       return "MaxisISD::MAQ_S_W_PHR";
  case MaxisISD::MAQ_SA_W_PHL:      return "MaxisISD::MAQ_SA_W_PHL";
  case MaxisISD::MAQ_SA_W_PHR:      return "MaxisISD::MAQ_SA_W_PHR";
  case MaxisISD::DPAU_H_QBL:        return "MaxisISD::DPAU_H_QBL";
  case MaxisISD::DPAU_H_QBR:        return "MaxisISD::DPAU_H_QBR";
  case MaxisISD::DPSU_H_QBL:        return "MaxisISD::DPSU_H_QBL";
  case MaxisISD::DPSU_H_QBR:        return "MaxisISD::DPSU_H_QBR";
  case MaxisISD::DPAQ_S_W_PH:       return "MaxisISD::DPAQ_S_W_PH";
  case MaxisISD::DPSQ_S_W_PH:       return "MaxisISD::DPSQ_S_W_PH";
  case MaxisISD::DPAQ_SA_L_W:       return "MaxisISD::DPAQ_SA_L_W";
  case MaxisISD::DPSQ_SA_L_W:       return "MaxisISD::DPSQ_SA_L_W";
  case MaxisISD::DPA_W_PH:          return "MaxisISD::DPA_W_PH";
  case MaxisISD::DPS_W_PH:          return "MaxisISD::DPS_W_PH";
  case MaxisISD::DPAQX_S_W_PH:      return "MaxisISD::DPAQX_S_W_PH";
  case MaxisISD::DPAQX_SA_W_PH:     return "MaxisISD::DPAQX_SA_W_PH";
  case MaxisISD::DPAX_W_PH:         return "MaxisISD::DPAX_W_PH";
  case MaxisISD::DPSX_W_PH:         return "MaxisISD::DPSX_W_PH";
  case MaxisISD::DPSQX_S_W_PH:      return "MaxisISD::DPSQX_S_W_PH";
  case MaxisISD::DPSQX_SA_W_PH:     return "MaxisISD::DPSQX_SA_W_PH";
  case MaxisISD::MULSA_W_PH:        return "MaxisISD::MULSA_W_PH";
  case MaxisISD::MULT:              return "MaxisISD::MULT";
  case MaxisISD::MULTU:             return "MaxisISD::MULTU";
  case MaxisISD::MADD_DSP:          return "MaxisISD::MADD_DSP";
  case MaxisISD::MADDU_DSP:         return "MaxisISD::MADDU_DSP";
  case MaxisISD::MSUB_DSP:          return "MaxisISD::MSUB_DSP";
  case MaxisISD::MSUBU_DSP:         return "MaxisISD::MSUBU_DSP";
  case MaxisISD::SHLL_DSP:          return "MaxisISD::SHLL_DSP";
  case MaxisISD::SHRA_DSP:          return "MaxisISD::SHRA_DSP";
  case MaxisISD::SHRL_DSP:          return "MaxisISD::SHRL_DSP";
  case MaxisISD::SETCC_DSP:         return "MaxisISD::SETCC_DSP";
  case MaxisISD::SELECT_CC_DSP:     return "MaxisISD::SELECT_CC_DSP";
  case MaxisISD::VALL_ZERO:         return "MaxisISD::VALL_ZERO";
  case MaxisISD::VANY_ZERO:         return "MaxisISD::VANY_ZERO";
  case MaxisISD::VALL_NONZERO:      return "MaxisISD::VALL_NONZERO";
  case MaxisISD::VANY_NONZERO:      return "MaxisISD::VANY_NONZERO";
  case MaxisISD::VCEQ:              return "MaxisISD::VCEQ";
  case MaxisISD::VCLE_S:            return "MaxisISD::VCLE_S";
  case MaxisISD::VCLE_U:            return "MaxisISD::VCLE_U";
  case MaxisISD::VCLT_S:            return "MaxisISD::VCLT_S";
  case MaxisISD::VCLT_U:            return "MaxisISD::VCLT_U";
  case MaxisISD::VSMAX:             return "MaxisISD::VSMAX";
  case MaxisISD::VSMIN:             return "MaxisISD::VSMIN";
  case MaxisISD::VUMAX:             return "MaxisISD::VUMAX";
  case MaxisISD::VUMIN:             return "MaxisISD::VUMIN";
  case MaxisISD::VEXTRACT_SEXT_ELT: return "MaxisISD::VEXTRACT_SEXT_ELT";
  case MaxisISD::VEXTRACT_ZEXT_ELT: return "MaxisISD::VEXTRACT_ZEXT_ELT";
  case MaxisISD::VNOR:              return "MaxisISD::VNOR";
  case MaxisISD::VSHF:              return "MaxisISD::VSHF";
  case MaxisISD::SHF:               return "MaxisISD::SHF";
  case MaxisISD::ILVEV:             return "MaxisISD::ILVEV";
  case MaxisISD::ILVOD:             return "MaxisISD::ILVOD";
  case MaxisISD::ILVL:              return "MaxisISD::ILVL";
  case MaxisISD::ILVR:              return "MaxisISD::ILVR";
  case MaxisISD::PCKEV:             return "MaxisISD::PCKEV";
  case MaxisISD::PCKOD:             return "MaxisISD::PCKOD";
  case MaxisISD::INSVE:             return "MaxisISD::INSVE";
  }
  return nullptr;
}

MaxisTargetLowering::MaxisTargetLowering(const MaxisTargetMachine &TM,
                                       const MaxisSubtarget &STI)
    : TargetLowering(TM), Subtarget(STI), ABI(TM.getABI()) {
  // Maxis does not have i1 type, so use i32 for
  // setcc operations results (slt, sgt, ...).
  setBooleanContents(ZeroOrOneBooleanContent);
  setBooleanVectorContents(ZeroOrNegativeOneBooleanContent);
  // The cmp.cond.fmt instruction in MAXIS32r6/MAXIS64r6 uses 0 and -1 like MSA
  // does. Integer booleans still use 0 and 1.
  if (Subtarget.hasMaxis32r6())
    setBooleanContents(ZeroOrOneBooleanContent,
                       ZeroOrNegativeOneBooleanContent);

  // Load extented operations for i1 types must be promoted
  for (MVT VT : MVT::integer_valuetypes()) {
    setLoadExtAction(ISD::EXTLOAD,  VT, MVT::i1,  Promote);
    setLoadExtAction(ISD::ZEXTLOAD, VT, MVT::i1,  Promote);
    setLoadExtAction(ISD::SEXTLOAD, VT, MVT::i1,  Promote);
  }

  // MAXIS doesn't have extending float->double load/store.  Set LoadExtAction
  // for f32, f16
  for (MVT VT : MVT::fp_valuetypes()) {
    setLoadExtAction(ISD::EXTLOAD, VT, MVT::f32, Expand);
    setLoadExtAction(ISD::EXTLOAD, VT, MVT::f16, Expand);
  }

  // Set LoadExtAction for f16 vectors to Expand
  for (MVT VT : MVT::fp_vector_valuetypes()) {
    MVT F16VT = MVT::getVectorVT(MVT::f16, VT.getVectorNumElements());
    if (F16VT.isValid())
      setLoadExtAction(ISD::EXTLOAD, VT, F16VT, Expand);
  }

  setTruncStoreAction(MVT::f32, MVT::f16, Expand);
  setTruncStoreAction(MVT::f64, MVT::f16, Expand);

  setTruncStoreAction(MVT::f64, MVT::f32, Expand);

  // Used by legalize types to correctly generate the setcc result.
  // Without this, every float setcc comes with a AND/OR with the result,
  // we don't want this, since the fpcmp result goes to a flag register,
  // which is used implicitly by brcond and select operations.
  AddPromotedToType(ISD::SETCC, MVT::i1, MVT::i32);

  // Maxis Custom Operations
  setOperationAction(ISD::BR_JT,              MVT::Other, Expand);
  setOperationAction(ISD::GlobalAddress,      MVT::i32,   Custom);
  setOperationAction(ISD::BlockAddress,       MVT::i32,   Custom);
  setOperationAction(ISD::GlobalTLSAddress,   MVT::i32,   Custom);
  setOperationAction(ISD::JumpTable,          MVT::i32,   Custom);
  setOperationAction(ISD::ConstantPool,       MVT::i32,   Custom);
  setOperationAction(ISD::SELECT,             MVT::f32,   Custom);
  setOperationAction(ISD::SELECT,             MVT::f64,   Custom);
  setOperationAction(ISD::SELECT,             MVT::i32,   Custom);
  setOperationAction(ISD::SETCC,              MVT::f32,   Custom);
  setOperationAction(ISD::SETCC,              MVT::f64,   Custom);
  setOperationAction(ISD::BRCOND,             MVT::Other, Custom);
  setOperationAction(ISD::FCOPYSIGN,          MVT::f32,   Custom);
  setOperationAction(ISD::FCOPYSIGN,          MVT::f64,   Custom);
  setOperationAction(ISD::FP_TO_SINT,         MVT::i32,   Custom);

  if (Subtarget.isGP64bit()) {
    setOperationAction(ISD::GlobalAddress,      MVT::i64,   Custom);
    setOperationAction(ISD::BlockAddress,       MVT::i64,   Custom);
    setOperationAction(ISD::GlobalTLSAddress,   MVT::i64,   Custom);
    setOperationAction(ISD::JumpTable,          MVT::i64,   Custom);
    setOperationAction(ISD::ConstantPool,       MVT::i64,   Custom);
    setOperationAction(ISD::SELECT,             MVT::i64,   Custom);
    setOperationAction(ISD::LOAD,               MVT::i64,   Custom);
    setOperationAction(ISD::STORE,              MVT::i64,   Custom);
    setOperationAction(ISD::FP_TO_SINT,         MVT::i64,   Custom);
    setOperationAction(ISD::SHL_PARTS,          MVT::i64,   Custom);
    setOperationAction(ISD::SRA_PARTS,          MVT::i64,   Custom);
    setOperationAction(ISD::SRL_PARTS,          MVT::i64,   Custom);
  }

  if (!Subtarget.isGP64bit()) {
    setOperationAction(ISD::SHL_PARTS,          MVT::i32,   Custom);
    setOperationAction(ISD::SRA_PARTS,          MVT::i32,   Custom);
    setOperationAction(ISD::SRL_PARTS,          MVT::i32,   Custom);
  }

  setOperationAction(ISD::EH_DWARF_CFA,         MVT::i32,   Custom);
  if (Subtarget.isGP64bit())
    setOperationAction(ISD::EH_DWARF_CFA,       MVT::i64,   Custom);

  setOperationAction(ISD::SDIV, MVT::i32, Expand);
  setOperationAction(ISD::SREM, MVT::i32, Expand);
  setOperationAction(ISD::UDIV, MVT::i32, Expand);
  setOperationAction(ISD::UREM, MVT::i32, Expand);
  setOperationAction(ISD::SDIV, MVT::i64, Expand);
  setOperationAction(ISD::SREM, MVT::i64, Expand);
  setOperationAction(ISD::UDIV, MVT::i64, Expand);
  setOperationAction(ISD::UREM, MVT::i64, Expand);

  if (!(Subtarget.hasDSP() && Subtarget.hasMaxis32r2())) {
    setOperationAction(ISD::ADDC, MVT::i32, Expand);
    setOperationAction(ISD::ADDE, MVT::i32, Expand);
  }

  setOperationAction(ISD::ADDC, MVT::i64, Expand);
  setOperationAction(ISD::ADDE, MVT::i64, Expand);
  setOperationAction(ISD::SUBC, MVT::i32, Expand);
  setOperationAction(ISD::SUBE, MVT::i32, Expand);
  setOperationAction(ISD::SUBC, MVT::i64, Expand);
  setOperationAction(ISD::SUBE, MVT::i64, Expand);

  // Operations not directly supported by Maxis.
  setOperationAction(ISD::BR_CC,             MVT::f32,   Expand);
  setOperationAction(ISD::BR_CC,             MVT::f64,   Expand);
  setOperationAction(ISD::BR_CC,             MVT::i32,   Expand);
  setOperationAction(ISD::BR_CC,             MVT::i64,   Expand);
  setOperationAction(ISD::SELECT_CC,         MVT::i32,   Expand);
  setOperationAction(ISD::SELECT_CC,         MVT::i64,   Expand);
  setOperationAction(ISD::SELECT_CC,         MVT::f32,   Expand);
  setOperationAction(ISD::SELECT_CC,         MVT::f64,   Expand);
  setOperationAction(ISD::UINT_TO_FP,        MVT::i32,   Expand);
  setOperationAction(ISD::UINT_TO_FP,        MVT::i64,   Expand);
  setOperationAction(ISD::FP_TO_UINT,        MVT::i32,   Expand);
  setOperationAction(ISD::FP_TO_UINT,        MVT::i64,   Expand);
  setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i1,    Expand);
  if (Subtarget.hasCnMaxis()) {
    setOperationAction(ISD::CTPOP,           MVT::i32,   Legal);
    setOperationAction(ISD::CTPOP,           MVT::i64,   Legal);
  } else {
    setOperationAction(ISD::CTPOP,           MVT::i32,   Expand);
    setOperationAction(ISD::CTPOP,           MVT::i64,   Expand);
  }
  setOperationAction(ISD::CTTZ,              MVT::i32,   Expand);
  setOperationAction(ISD::CTTZ,              MVT::i64,   Expand);
  setOperationAction(ISD::ROTL,              MVT::i32,   Expand);
  setOperationAction(ISD::ROTL,              MVT::i64,   Expand);
  setOperationAction(ISD::DYNAMIC_STACKALLOC, MVT::i32,  Expand);
  setOperationAction(ISD::DYNAMIC_STACKALLOC, MVT::i64,  Expand);

  if (!Subtarget.hasMaxis32r2())
    setOperationAction(ISD::ROTR, MVT::i32,   Expand);

  if (!Subtarget.hasMaxis64r2())
    setOperationAction(ISD::ROTR, MVT::i64,   Expand);

  setOperationAction(ISD::FSIN,              MVT::f32,   Expand);
  setOperationAction(ISD::FSIN,              MVT::f64,   Expand);
  setOperationAction(ISD::FCOS,              MVT::f32,   Expand);
  setOperationAction(ISD::FCOS,              MVT::f64,   Expand);
  setOperationAction(ISD::FSINCOS,           MVT::f32,   Expand);
  setOperationAction(ISD::FSINCOS,           MVT::f64,   Expand);
  setOperationAction(ISD::FPOW,              MVT::f32,   Expand);
  setOperationAction(ISD::FPOW,              MVT::f64,   Expand);
  setOperationAction(ISD::FLOG,              MVT::f32,   Expand);
  setOperationAction(ISD::FLOG2,             MVT::f32,   Expand);
  setOperationAction(ISD::FLOG10,            MVT::f32,   Expand);
  setOperationAction(ISD::FEXP,              MVT::f32,   Expand);
  setOperationAction(ISD::FMA,               MVT::f32,   Expand);
  setOperationAction(ISD::FMA,               MVT::f64,   Expand);
  setOperationAction(ISD::FREM,              MVT::f32,   Expand);
  setOperationAction(ISD::FREM,              MVT::f64,   Expand);

  // Lower f16 conversion operations into library calls
  setOperationAction(ISD::FP16_TO_FP,        MVT::f32,   Expand);
  setOperationAction(ISD::FP_TO_FP16,        MVT::f32,   Expand);
  setOperationAction(ISD::FP16_TO_FP,        MVT::f64,   Expand);
  setOperationAction(ISD::FP_TO_FP16,        MVT::f64,   Expand);

  setOperationAction(ISD::EH_RETURN, MVT::Other, Custom);

  setOperationAction(ISD::VASTART,           MVT::Other, Custom);
  setOperationAction(ISD::VAARG,             MVT::Other, Custom);
  setOperationAction(ISD::VACOPY,            MVT::Other, Expand);
  setOperationAction(ISD::VAEND,             MVT::Other, Expand);

  // Use the default for now
  setOperationAction(ISD::STACKSAVE,         MVT::Other, Expand);
  setOperationAction(ISD::STACKRESTORE,      MVT::Other, Expand);

  if (!Subtarget.isGP64bit()) {
    setOperationAction(ISD::ATOMIC_LOAD,     MVT::i64,   Expand);
    setOperationAction(ISD::ATOMIC_STORE,    MVT::i64,   Expand);
  }

  if (!Subtarget.hasMaxis32r2()) {
    setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i8,  Expand);
    setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i16, Expand);
  }

  // MAXIS16 lacks MAXIS32's clz and clo instructions.
  if (!Subtarget.hasMaxis32() || Subtarget.inMaxis16Mode())
    setOperationAction(ISD::CTLZ, MVT::i32, Expand);
  if (!Subtarget.hasMaxis64())
    setOperationAction(ISD::CTLZ, MVT::i64, Expand);

  if (!Subtarget.hasMaxis32r2())
    setOperationAction(ISD::BSWAP, MVT::i32, Expand);
  if (!Subtarget.hasMaxis64r2())
    setOperationAction(ISD::BSWAP, MVT::i64, Expand);

  if (Subtarget.isGP64bit()) {
    setLoadExtAction(ISD::SEXTLOAD, MVT::i64, MVT::i32, Custom);
    setLoadExtAction(ISD::ZEXTLOAD, MVT::i64, MVT::i32, Custom);
    setLoadExtAction(ISD::EXTLOAD, MVT::i64, MVT::i32, Custom);
    setTruncStoreAction(MVT::i64, MVT::i32, Custom);
  }

  setOperationAction(ISD::TRAP, MVT::Other, Legal);

  setTargetDAGCombine(ISD::SDIVREM);
  setTargetDAGCombine(ISD::UDIVREM);
  setTargetDAGCombine(ISD::SELECT);
  setTargetDAGCombine(ISD::AND);
  setTargetDAGCombine(ISD::OR);
  setTargetDAGCombine(ISD::ADD);
  setTargetDAGCombine(ISD::SUB);
  setTargetDAGCombine(ISD::AssertZext);
  setTargetDAGCombine(ISD::SHL);

  if (ABI.IsO32()) {
    // These libcalls are not available in 32-bit.
    setLibcallName(RTLIB::SHL_I128, nullptr);
    setLibcallName(RTLIB::SRL_I128, nullptr);
    setLibcallName(RTLIB::SRA_I128, nullptr);
  }

  setMinFunctionAlignment(Subtarget.isGP64bit() ? 3 : 2);

  // The arguments on the stack are defined in terms of 4-byte slots on O32
  // and 8-byte slots on N32/N64.
  setMinStackArgumentAlignment((ABI.IsN32() || ABI.IsN64()) ? 8 : 4);

  setStackPointerRegisterToSaveRestore(ABI.IsN64() ? Maxis::SP_64 : Maxis::SP);

  MaxStoresPerMemcpy = 16;

  isMicroMaxis = Subtarget.inMicroMaxisMode();
}

const MaxisTargetLowering *MaxisTargetLowering::create(const MaxisTargetMachine &TM,
                                                     const MaxisSubtarget &STI) {
  if (STI.inMaxis16Mode())
    return createMaxis16TargetLowering(TM, STI);

  return createMaxisSETargetLowering(TM, STI);
}

// Create a fast isel object.
FastISel *
MaxisTargetLowering::createFastISel(FunctionLoweringInfo &funcInfo,
                                  const TargetLibraryInfo *libInfo) const {
  const MaxisTargetMachine &TM =
      static_cast<const MaxisTargetMachine &>(funcInfo.MF->getTarget());

  // We support only the standard encoding [MAXIS32,MAXIS32R5] ISAs.
  bool UseFastISel = TM.Options.EnableFastISel && Subtarget.hasMaxis32() &&
                     !Subtarget.hasMaxis32r6() && !Subtarget.inMaxis16Mode() &&
                     !Subtarget.inMicroMaxisMode();

  // Disable if either of the following is true:
  // We do not generate PIC, the ABI is not O32, LargeGOT is being used.
  if (!TM.isPositionIndependent() || !TM.getABI().IsO32() || LargeGOT)
    UseFastISel = false;

  return UseFastISel ? Maxis::createFastISel(funcInfo, libInfo) : nullptr;
}

EVT MaxisTargetLowering::getSetCCResultType(const DataLayout &, LLVMContext &,
                                           EVT VT) const {
  if (!VT.isVector())
    return MVT::i32;
  return VT.changeVectorElementTypeToInteger();
}

static SDValue performDivRemCombine(SDNode *N, SelectionDAG &DAG,
                                    TargetLowering::DAGCombinerInfo &DCI,
                                    const MaxisSubtarget &Subtarget) {
  if (DCI.isBeforeLegalizeOps())
    return SDValue();

  EVT Ty = N->getValueType(0);
  unsigned LO = (Ty == MVT::i32) ? Maxis::LO0 : Maxis::LO0_64;
  unsigned HI = (Ty == MVT::i32) ? Maxis::HI0 : Maxis::HI0_64;
  unsigned Opc = N->getOpcode() == ISD::SDIVREM ? MaxisISD::DivRem16 :
                                                  MaxisISD::DivRemU16;
  SDLoc DL(N);

  SDValue DivRem = DAG.getNode(Opc, DL, MVT::Glue,
                               N->getOperand(0), N->getOperand(1));
  SDValue InChain = DAG.getEntryNode();
  SDValue InGlue = DivRem;

  // insert MFLO
  if (N->hasAnyUseOfValue(0)) {
    SDValue CopyFromLo = DAG.getCopyFromReg(InChain, DL, LO, Ty,
                                            InGlue);
    DAG.ReplaceAllUsesOfValueWith(SDValue(N, 0), CopyFromLo);
    InChain = CopyFromLo.getValue(1);
    InGlue = CopyFromLo.getValue(2);
  }

  // insert MFHI
  if (N->hasAnyUseOfValue(1)) {
    SDValue CopyFromHi = DAG.getCopyFromReg(InChain, DL,
                                            HI, Ty, InGlue);
    DAG.ReplaceAllUsesOfValueWith(SDValue(N, 1), CopyFromHi);
  }

  return SDValue();
}

static Maxis::CondCode condCodeToFCC(ISD::CondCode CC) {
  switch (CC) {
  default: llvm_unreachable("Unknown fp condition code!");
  case ISD::SETEQ:
  case ISD::SETOEQ: return Maxis::FCOND_OEQ;
  case ISD::SETUNE: return Maxis::FCOND_UNE;
  case ISD::SETLT:
  case ISD::SETOLT: return Maxis::FCOND_OLT;
  case ISD::SETGT:
  case ISD::SETOGT: return Maxis::FCOND_OGT;
  case ISD::SETLE:
  case ISD::SETOLE: return Maxis::FCOND_OLE;
  case ISD::SETGE:
  case ISD::SETOGE: return Maxis::FCOND_OGE;
  case ISD::SETULT: return Maxis::FCOND_ULT;
  case ISD::SETULE: return Maxis::FCOND_ULE;
  case ISD::SETUGT: return Maxis::FCOND_UGT;
  case ISD::SETUGE: return Maxis::FCOND_UGE;
  case ISD::SETUO:  return Maxis::FCOND_UN;
  case ISD::SETO:   return Maxis::FCOND_OR;
  case ISD::SETNE:
  case ISD::SETONE: return Maxis::FCOND_ONE;
  case ISD::SETUEQ: return Maxis::FCOND_UEQ;
  }
}

/// This function returns true if the floating point conditional branches and
/// conditional moves which use condition code CC should be inverted.
static bool invertFPCondCodeUser(Maxis::CondCode CC) {
  if (CC >= Maxis::FCOND_F && CC <= Maxis::FCOND_NGT)
    return false;

  assert((CC >= Maxis::FCOND_T && CC <= Maxis::FCOND_GT) &&
         "Illegal Condition Code");

  return true;
}

// Creates and returns an FPCmp node from a setcc node.
// Returns Op if setcc is not a floating point comparison.
static SDValue createFPCmp(SelectionDAG &DAG, const SDValue &Op) {
  // must be a SETCC node
  if (Op.getOpcode() != ISD::SETCC)
    return Op;

  SDValue LHS = Op.getOperand(0);

  if (!LHS.getValueType().isFloatingPoint())
    return Op;

  SDValue RHS = Op.getOperand(1);
  SDLoc DL(Op);

  // Assume the 3rd operand is a CondCodeSDNode. Add code to check the type of
  // node if necessary.
  ISD::CondCode CC = cast<CondCodeSDNode>(Op.getOperand(2))->get();

  return DAG.getNode(MaxisISD::FPCmp, DL, MVT::Glue, LHS, RHS,
                     DAG.getConstant(condCodeToFCC(CC), DL, MVT::i32));
}

// Creates and returns a CMovFPT/F node.
static SDValue createCMovFP(SelectionDAG &DAG, SDValue Cond, SDValue True,
                            SDValue False, const SDLoc &DL) {
  ConstantSDNode *CC = cast<ConstantSDNode>(Cond.getOperand(2));
  bool invert = invertFPCondCodeUser((Maxis::CondCode)CC->getSExtValue());
  SDValue FCC0 = DAG.getRegister(Maxis::FCC0, MVT::i32);

  return DAG.getNode((invert ? MaxisISD::CMovFP_F : MaxisISD::CMovFP_T), DL,
                     True.getValueType(), True, FCC0, False, Cond);
}

static SDValue performSELECTCombine(SDNode *N, SelectionDAG &DAG,
                                    TargetLowering::DAGCombinerInfo &DCI,
                                    const MaxisSubtarget &Subtarget) {
  if (DCI.isBeforeLegalizeOps())
    return SDValue();

  SDValue SetCC = N->getOperand(0);

  if ((SetCC.getOpcode() != ISD::SETCC) ||
      !SetCC.getOperand(0).getValueType().isInteger())
    return SDValue();

  SDValue False = N->getOperand(2);
  EVT FalseTy = False.getValueType();

  if (!FalseTy.isInteger())
    return SDValue();

  ConstantSDNode *FalseC = dyn_cast<ConstantSDNode>(False);

  // If the RHS (False) is 0, we swap the order of the operands
  // of ISD::SELECT (obviously also inverting the condition) so that we can
  // take advantage of conditional moves using the $0 register.
  // Example:
  //   return (a != 0) ? x : 0;
  //     load $reg, x
  //     movz $reg, $0, a
  if (!FalseC)
    return SDValue();

  const SDLoc DL(N);

  if (!FalseC->getZExtValue()) {
    ISD::CondCode CC = cast<CondCodeSDNode>(SetCC.getOperand(2))->get();
    SDValue True = N->getOperand(1);

    SetCC = DAG.getSetCC(DL, SetCC.getValueType(), SetCC.getOperand(0),
                         SetCC.getOperand(1), ISD::getSetCCInverse(CC, true));

    return DAG.getNode(ISD::SELECT, DL, FalseTy, SetCC, False, True);
  }

  // If both operands are integer constants there's a possibility that we
  // can do some interesting optimizations.
  SDValue True = N->getOperand(1);
  ConstantSDNode *TrueC = dyn_cast<ConstantSDNode>(True);

  if (!TrueC || !True.getValueType().isInteger())
    return SDValue();

  // We'll also ignore MVT::i64 operands as this optimizations proves
  // to be ineffective because of the required sign extensions as the result
  // of a SETCC operator is always MVT::i32 for non-vector types.
  if (True.getValueType() == MVT::i64)
    return SDValue();

  int64_t Diff = TrueC->getSExtValue() - FalseC->getSExtValue();

  // 1)  (a < x) ? y : y-1
  //  slti $reg1, a, x
  //  addi $reg2, $reg1, y-1
  if (Diff == 1)
    return DAG.getNode(ISD::ADD, DL, SetCC.getValueType(), SetCC, False);

  // 2)  (a < x) ? y-1 : y
  //  slti $reg1, a, x
  //  xor $reg1, $reg1, 1
  //  addi $reg2, $reg1, y-1
  if (Diff == -1) {
    ISD::CondCode CC = cast<CondCodeSDNode>(SetCC.getOperand(2))->get();
    SetCC = DAG.getSetCC(DL, SetCC.getValueType(), SetCC.getOperand(0),
                         SetCC.getOperand(1), ISD::getSetCCInverse(CC, true));
    return DAG.getNode(ISD::ADD, DL, SetCC.getValueType(), SetCC, True);
  }

  // Couldn't optimize.
  return SDValue();
}

static SDValue performCMovFPCombine(SDNode *N, SelectionDAG &DAG,
                                    TargetLowering::DAGCombinerInfo &DCI,
                                    const MaxisSubtarget &Subtarget) {
  if (DCI.isBeforeLegalizeOps())
    return SDValue();

  SDValue ValueIfTrue = N->getOperand(0), ValueIfFalse = N->getOperand(2);

  ConstantSDNode *FalseC = dyn_cast<ConstantSDNode>(ValueIfFalse);
  if (!FalseC || FalseC->getZExtValue())
    return SDValue();

  // Since RHS (False) is 0, we swap the order of the True/False operands
  // (obviously also inverting the condition) so that we can
  // take advantage of conditional moves using the $0 register.
  // Example:
  //   return (a != 0) ? x : 0;
  //     load $reg, x
  //     movz $reg, $0, a
  unsigned Opc = (N->getOpcode() == MaxisISD::CMovFP_T) ? MaxisISD::CMovFP_F :
                                                         MaxisISD::CMovFP_T;

  SDValue FCC = N->getOperand(1), Glue = N->getOperand(3);
  return DAG.getNode(Opc, SDLoc(N), ValueIfFalse.getValueType(),
                     ValueIfFalse, FCC, ValueIfTrue, Glue);
}

static SDValue performANDCombine(SDNode *N, SelectionDAG &DAG,
                                 TargetLowering::DAGCombinerInfo &DCI,
                                 const MaxisSubtarget &Subtarget) {
  if (DCI.isBeforeLegalizeOps() || !Subtarget.hasExtractInsert())
    return SDValue();

  SDValue FirstOperand = N->getOperand(0);
  unsigned FirstOperandOpc = FirstOperand.getOpcode();
  SDValue Mask = N->getOperand(1);
  EVT ValTy = N->getValueType(0);
  SDLoc DL(N);

  uint64_t Pos = 0, SMPos, SMSize;
  ConstantSDNode *CN;
  SDValue NewOperand;
  unsigned Opc;

  // Op's second operand must be a shifted mask.
  if (!(CN = dyn_cast<ConstantSDNode>(Mask)) ||
      !isShiftedMask(CN->getZExtValue(), SMPos, SMSize))
    return SDValue();

  if (FirstOperandOpc == ISD::SRA || FirstOperandOpc == ISD::SRL) {
    // Pattern match EXT.
    //  $dst = and ((sra or srl) $src , pos), (2**size - 1)
    //  => ext $dst, $src, pos, size

    // The second operand of the shift must be an immediate.
    if (!(CN = dyn_cast<ConstantSDNode>(FirstOperand.getOperand(1))))
      return SDValue();

    Pos = CN->getZExtValue();

    // Return if the shifted mask does not start at bit 0 or the sum of its size
    // and Pos exceeds the word's size.
    if (SMPos != 0 || Pos + SMSize > ValTy.getSizeInBits())
      return SDValue();

    Opc = MaxisISD::Ext;
    NewOperand = FirstOperand.getOperand(0);
  } else if (FirstOperandOpc == ISD::SHL && Subtarget.hasCnMaxis()) {
    // Pattern match CINS.
    //  $dst = and (shl $src , pos), mask
    //  => cins $dst, $src, pos, size
    // mask is a shifted mask with consecutive 1's, pos = shift amount,
    // size = population count.

    // The second operand of the shift must be an immediate.
    if (!(CN = dyn_cast<ConstantSDNode>(FirstOperand.getOperand(1))))
      return SDValue();

    Pos = CN->getZExtValue();

    if (SMPos != Pos || Pos >= ValTy.getSizeInBits() || SMSize >= 32 ||
        Pos + SMSize > ValTy.getSizeInBits())
      return SDValue();

    NewOperand = FirstOperand.getOperand(0);
    // SMSize is 'location' (position) in this case, not size.
    SMSize--;
    Opc = MaxisISD::CIns;
  } else {
    // Pattern match EXT.
    //  $dst = and $src, (2**size - 1) , if size > 16
    //  => ext $dst, $src, pos, size , pos = 0

    // If the mask is <= 0xffff, andi can be used instead.
    if (CN->getZExtValue() <= 0xffff)
      return SDValue();

    // Return if the mask doesn't start at position 0.
    if (SMPos)
      return SDValue();

    Opc = MaxisISD::Ext;
    NewOperand = FirstOperand;
  }
  return DAG.getNode(Opc, DL, ValTy, NewOperand,
                     DAG.getConstant(Pos, DL, MVT::i32),
                     DAG.getConstant(SMSize, DL, MVT::i32));
}

static SDValue performORCombine(SDNode *N, SelectionDAG &DAG,
                                TargetLowering::DAGCombinerInfo &DCI,
                                const MaxisSubtarget &Subtarget) {
  // Pattern match INS.
  //  $dst = or (and $src1 , mask0), (and (shl $src, pos), mask1),
  //  where mask1 = (2**size - 1) << pos, mask0 = ~mask1
  //  => ins $dst, $src, size, pos, $src1
  if (DCI.isBeforeLegalizeOps() || !Subtarget.hasExtractInsert())
    return SDValue();

  SDValue And0 = N->getOperand(0), And1 = N->getOperand(1);
  uint64_t SMPos0, SMSize0, SMPos1, SMSize1;
  ConstantSDNode *CN, *CN1;

  // See if Op's first operand matches (and $src1 , mask0).
  if (And0.getOpcode() != ISD::AND)
    return SDValue();

  if (!(CN = dyn_cast<ConstantSDNode>(And0.getOperand(1))) ||
      !isShiftedMask(~CN->getSExtValue(), SMPos0, SMSize0))
    return SDValue();

  // See if Op's second operand matches (and (shl $src, pos), mask1).
  if (And1.getOpcode() == ISD::AND &&
      And1.getOperand(0).getOpcode() == ISD::SHL) {

    if (!(CN = dyn_cast<ConstantSDNode>(And1.getOperand(1))) ||
        !isShiftedMask(CN->getZExtValue(), SMPos1, SMSize1))
      return SDValue();

    // The shift masks must have the same position and size.
    if (SMPos0 != SMPos1 || SMSize0 != SMSize1)
      return SDValue();

    SDValue Shl = And1.getOperand(0);

    if (!(CN = dyn_cast<ConstantSDNode>(Shl.getOperand(1))))
      return SDValue();

    unsigned Shamt = CN->getZExtValue();

    // Return if the shift amount and the first bit position of mask are not the
    // same.
    EVT ValTy = N->getValueType(0);
    if ((Shamt != SMPos0) || (SMPos0 + SMSize0 > ValTy.getSizeInBits()))
      return SDValue();

    SDLoc DL(N);
    return DAG.getNode(MaxisISD::Ins, DL, ValTy, Shl.getOperand(0),
                       DAG.getConstant(SMPos0, DL, MVT::i32),
                       DAG.getConstant(SMSize0, DL, MVT::i32),
                       And0.getOperand(0));
  } else {
    // Pattern match DINS.
    //  $dst = or (and $src, mask0), mask1
    //  where mask0 = ((1 << SMSize0) -1) << SMPos0
    //  => dins $dst, $src, pos, size
    if (~CN->getSExtValue() == ((((int64_t)1 << SMSize0) - 1) << SMPos0) &&
        ((SMSize0 + SMPos0 <= 64 && Subtarget.hasMaxis64r2()) ||
         (SMSize0 + SMPos0 <= 32))) {
      // Check if AND instruction has constant as argument
      bool isConstCase = And1.getOpcode() != ISD::AND;
      if (And1.getOpcode() == ISD::AND) {
        if (!(CN1 = dyn_cast<ConstantSDNode>(And1->getOperand(1))))
          return SDValue();
      } else {
        if (!(CN1 = dyn_cast<ConstantSDNode>(N->getOperand(1))))
          return SDValue();
      }
      // Don't generate INS if constant OR operand doesn't fit into bits
      // cleared by constant AND operand.
      if (CN->getSExtValue() & CN1->getSExtValue())
        return SDValue();

      SDLoc DL(N);
      EVT ValTy = N->getOperand(0)->getValueType(0);
      SDValue Const1;
      SDValue SrlX;
      if (!isConstCase) {
        Const1 = DAG.getConstant(SMPos0, DL, MVT::i32);
        SrlX = DAG.getNode(ISD::SRL, DL, And1->getValueType(0), And1, Const1);
      }
      return DAG.getNode(
          MaxisISD::Ins, DL, N->getValueType(0),
          isConstCase
              ? DAG.getConstant(CN1->getSExtValue() >> SMPos0, DL, ValTy)
              : SrlX,
          DAG.getConstant(SMPos0, DL, MVT::i32),
          DAG.getConstant(ValTy.getSizeInBits() / 8 < 8 ? SMSize0 & 31
                                                        : SMSize0,
                          DL, MVT::i32),
          And0->getOperand(0));

    }
    return SDValue();
  }
}

static SDValue performMADD_MSUBCombine(SDNode *ROOTNode, SelectionDAG &CurDAG,
                                       const MaxisSubtarget &Subtarget) {
  // ROOTNode must have a multiplication as an operand for the match to be
  // successful.
  if (ROOTNode->getOperand(0).getOpcode() != ISD::MUL &&
      ROOTNode->getOperand(1).getOpcode() != ISD::MUL)
    return SDValue();

  // We don't handle vector types here.
  if (ROOTNode->getValueType(0).isVector())
    return SDValue();

  // For MAXIS64, madd / msub instructions are inefficent to use with 64 bit
  // arithmetic. E.g.
  // (add (mul a b) c) =>
  //   let res = (madd (mthi (drotr c 32))x(mtlo c) a b) in
  //   MAXIS64:   (or (dslli (mfhi res) 32) (dsrl (dslli (mflo res) 32) 32)
  //   or
  //   MAXIS64R2: (dins (mflo res) (mfhi res) 32 32)
  //
  // The overhead of setting up the Hi/Lo registers and reassembling the
  // result makes this a dubious optimzation for MAXIS64. The core of the
  // problem is that Hi/Lo contain the upper and lower 32 bits of the
  // operand and result.
  //
  // It requires a chain of 4 add/mul for MAXIS64R2 to get better code
  // density than doing it naively, 5 for MAXIS64. Additionally, using
  // madd/msub on MAXIS64 requires the operands actually be 32 bit sign
  // extended operands, not true 64 bit values.
  //
  // FIXME: For the moment, disable this completely for MAXIS64.
  if (Subtarget.hasMaxis64())
    return SDValue();

  SDValue Mult = ROOTNode->getOperand(0).getOpcode() == ISD::MUL
                     ? ROOTNode->getOperand(0)
                     : ROOTNode->getOperand(1);

  SDValue AddOperand = ROOTNode->getOperand(0).getOpcode() == ISD::MUL
                     ? ROOTNode->getOperand(1)
                     : ROOTNode->getOperand(0);

  // Transform this to a MADD only if the user of this node is the add.
  // If there are other users of the mul, this function returns here.
  if (!Mult.hasOneUse())
    return SDValue();

  // maddu and madd are unusual instructions in that on MAXIS64 bits 63..31
  // must be in canonical form, i.e. sign extended. For MAXIS32, the operands
  // of the multiply must have 32 or more sign bits, otherwise we cannot
  // perform this optimization. We have to check this here as we're performing
  // this optimization pre-legalization.
  SDValue MultLHS = Mult->getOperand(0);
  SDValue MultRHS = Mult->getOperand(1);

  bool IsSigned = MultLHS->getOpcode() == ISD::SIGN_EXTEND &&
                  MultRHS->getOpcode() == ISD::SIGN_EXTEND;
  bool IsUnsigned = MultLHS->getOpcode() == ISD::ZERO_EXTEND &&
                    MultRHS->getOpcode() == ISD::ZERO_EXTEND;

  if (!IsSigned && !IsUnsigned)
    return SDValue();

  // Initialize accumulator.
  SDLoc DL(ROOTNode);
  SDValue TopHalf;
  SDValue BottomHalf;
  BottomHalf = CurDAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i32, AddOperand,
                              CurDAG.getIntPtrConstant(0, DL));

  TopHalf = CurDAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i32, AddOperand,
                           CurDAG.getIntPtrConstant(1, DL));
  SDValue ACCIn = CurDAG.getNode(MaxisISD::MTLOHI, DL, MVT::Untyped,
                                  BottomHalf,
                                  TopHalf);

  // Create MaxisMAdd(u) / MaxisMSub(u) node.
  bool IsAdd = ROOTNode->getOpcode() == ISD::ADD;
  unsigned Opcode = IsAdd ? (IsUnsigned ? MaxisISD::MAddu : MaxisISD::MAdd)
                          : (IsUnsigned ? MaxisISD::MSubu : MaxisISD::MSub);
  SDValue MAddOps[3] = {
      CurDAG.getNode(ISD::TRUNCATE, DL, MVT::i32, Mult->getOperand(0)),
      CurDAG.getNode(ISD::TRUNCATE, DL, MVT::i32, Mult->getOperand(1)), ACCIn};
  EVT VTs[2] = {MVT::i32, MVT::i32};
  SDValue MAdd = CurDAG.getNode(Opcode, DL, VTs, MAddOps);

  SDValue ResLo = CurDAG.getNode(MaxisISD::MFLO, DL, MVT::i32, MAdd);
  SDValue ResHi = CurDAG.getNode(MaxisISD::MFHI, DL, MVT::i32, MAdd);
  SDValue Combined =
      CurDAG.getNode(ISD::BUILD_PAIR, DL, MVT::i64, ResLo, ResHi);
  return Combined;
}

static SDValue performSUBCombine(SDNode *N, SelectionDAG &DAG,
                                 TargetLowering::DAGCombinerInfo &DCI,
                                 const MaxisSubtarget &Subtarget) {
  // (sub v0 (mul v1, v2)) => (msub v1, v2, v0)
  if (DCI.isBeforeLegalizeOps()) {
    if (Subtarget.hasMaxis32() && !Subtarget.hasMaxis32r6() &&
        !Subtarget.inMaxis16Mode() && N->getValueType(0) == MVT::i64)
      return performMADD_MSUBCombine(N, DAG, Subtarget);

    return SDValue();
  }

  return SDValue();
}

static SDValue performADDCombine(SDNode *N, SelectionDAG &DAG,
                                 TargetLowering::DAGCombinerInfo &DCI,
                                 const MaxisSubtarget &Subtarget) {
  // (add v0 (mul v1, v2)) => (madd v1, v2, v0)
  if (DCI.isBeforeLegalizeOps()) {
    if (Subtarget.hasMaxis32() && !Subtarget.hasMaxis32r6() &&
        !Subtarget.inMaxis16Mode() && N->getValueType(0) == MVT::i64)
      return performMADD_MSUBCombine(N, DAG, Subtarget);

    return SDValue();
  }

  // (add v0, (add v1, abs_lo(tjt))) => (add (add v0, v1), abs_lo(tjt))
  SDValue Add = N->getOperand(1);

  if (Add.getOpcode() != ISD::ADD)
    return SDValue();

  SDValue Lo = Add.getOperand(1);

  if ((Lo.getOpcode() != MaxisISD::Lo) ||
      (Lo.getOperand(0).getOpcode() != ISD::TargetJumpTable))
    return SDValue();

  EVT ValTy = N->getValueType(0);
  SDLoc DL(N);

  SDValue Add1 = DAG.getNode(ISD::ADD, DL, ValTy, N->getOperand(0),
                             Add.getOperand(0));
  return DAG.getNode(ISD::ADD, DL, ValTy, Add1, Lo);
}

static SDValue performSHLCombine(SDNode *N, SelectionDAG &DAG,
                                 TargetLowering::DAGCombinerInfo &DCI,
                                 const MaxisSubtarget &Subtarget) {
  // Pattern match CINS.
  //  $dst = shl (and $src , imm), pos
  //  => cins $dst, $src, pos, size

  if (DCI.isBeforeLegalizeOps() || !Subtarget.hasCnMaxis())
    return SDValue();

  SDValue FirstOperand = N->getOperand(0);
  unsigned FirstOperandOpc = FirstOperand.getOpcode();
  SDValue SecondOperand = N->getOperand(1);
  EVT ValTy = N->getValueType(0);
  SDLoc DL(N);

  uint64_t Pos = 0, SMPos, SMSize;
  ConstantSDNode *CN;
  SDValue NewOperand;

  // The second operand of the shift must be an immediate.
  if (!(CN = dyn_cast<ConstantSDNode>(SecondOperand)))
    return SDValue();

  Pos = CN->getZExtValue();

  if (Pos >= ValTy.getSizeInBits())
    return SDValue();

  if (FirstOperandOpc != ISD::AND)
    return SDValue();

  // AND's second operand must be a shifted mask.
  if (!(CN = dyn_cast<ConstantSDNode>(FirstOperand.getOperand(1))) ||
      !isShiftedMask(CN->getZExtValue(), SMPos, SMSize))
    return SDValue();

  // Return if the shifted mask does not start at bit 0 or the sum of its size
  // and Pos exceeds the word's size.
  if (SMPos != 0 || SMSize > 32 || Pos + SMSize > ValTy.getSizeInBits())
    return SDValue();

  NewOperand = FirstOperand.getOperand(0);
  // SMSize is 'location' (position) in this case, not size.
  SMSize--;

  return DAG.getNode(MaxisISD::CIns, DL, ValTy, NewOperand,
                     DAG.getConstant(Pos, DL, MVT::i32),
                     DAG.getConstant(SMSize, DL, MVT::i32));
}

SDValue  MaxisTargetLowering::PerformDAGCombine(SDNode *N, DAGCombinerInfo &DCI)
  const {
  SelectionDAG &DAG = DCI.DAG;
  unsigned Opc = N->getOpcode();

  switch (Opc) {
  default: break;
  case ISD::SDIVREM:
  case ISD::UDIVREM:
    return performDivRemCombine(N, DAG, DCI, Subtarget);
  case ISD::SELECT:
    return performSELECTCombine(N, DAG, DCI, Subtarget);
  case MaxisISD::CMovFP_F:
  case MaxisISD::CMovFP_T:
    return performCMovFPCombine(N, DAG, DCI, Subtarget);
  case ISD::AND:
    return performANDCombine(N, DAG, DCI, Subtarget);
  case ISD::OR:
    return performORCombine(N, DAG, DCI, Subtarget);
  case ISD::ADD:
    return performADDCombine(N, DAG, DCI, Subtarget);
  case ISD::SHL:
    return performSHLCombine(N, DAG, DCI, Subtarget);
  case ISD::SUB:
    return performSUBCombine(N, DAG, DCI, Subtarget);
  }

  return SDValue();
}

bool MaxisTargetLowering::isCheapToSpeculateCttz() const {
  return Subtarget.hasMaxis32();
}

bool MaxisTargetLowering::isCheapToSpeculateCtlz() const {
  return Subtarget.hasMaxis32();
}

void
MaxisTargetLowering::LowerOperationWrapper(SDNode *N,
                                          SmallVectorImpl<SDValue> &Results,
                                          SelectionDAG &DAG) const {
  SDValue Res = LowerOperation(SDValue(N, 0), DAG);

  for (unsigned I = 0, E = Res->getNumValues(); I != E; ++I)
    Results.push_back(Res.getValue(I));
}

void
MaxisTargetLowering::ReplaceNodeResults(SDNode *N,
                                       SmallVectorImpl<SDValue> &Results,
                                       SelectionDAG &DAG) const {
  return LowerOperationWrapper(N, Results, DAG);
}

SDValue MaxisTargetLowering::
LowerOperation(SDValue Op, SelectionDAG &DAG) const
{
  switch (Op.getOpcode())
  {
  case ISD::BRCOND:             return lowerBRCOND(Op, DAG);
  case ISD::ConstantPool:       return lowerConstantPool(Op, DAG);
  case ISD::GlobalAddress:      return lowerGlobalAddress(Op, DAG);
  case ISD::BlockAddress:       return lowerBlockAddress(Op, DAG);
  case ISD::GlobalTLSAddress:   return lowerGlobalTLSAddress(Op, DAG);
  case ISD::JumpTable:          return lowerJumpTable(Op, DAG);
  case ISD::SELECT:             return lowerSELECT(Op, DAG);
  case ISD::SETCC:              return lowerSETCC(Op, DAG);
  case ISD::VASTART:            return lowerVASTART(Op, DAG);
  case ISD::VAARG:              return lowerVAARG(Op, DAG);
  case ISD::FCOPYSIGN:          return lowerFCOPYSIGN(Op, DAG);
  case ISD::FRAMEADDR:          return lowerFRAMEADDR(Op, DAG);
  case ISD::RETURNADDR:         return lowerRETURNADDR(Op, DAG);
  case ISD::EH_RETURN:          return lowerEH_RETURN(Op, DAG);
  case ISD::ATOMIC_FENCE:       return lowerATOMIC_FENCE(Op, DAG);
  case ISD::SHL_PARTS:          return lowerShiftLeftParts(Op, DAG);
  case ISD::SRA_PARTS:          return lowerShiftRightParts(Op, DAG, true);
  case ISD::SRL_PARTS:          return lowerShiftRightParts(Op, DAG, false);
  case ISD::LOAD:               return lowerLOAD(Op, DAG);
  case ISD::STORE:              return lowerSTORE(Op, DAG);
  case ISD::EH_DWARF_CFA:       return lowerEH_DWARF_CFA(Op, DAG);
  case ISD::FP_TO_SINT:         return lowerFP_TO_SINT(Op, DAG);
  }
  return SDValue();
}

//===----------------------------------------------------------------------===//
//  Lower helper functions
//===----------------------------------------------------------------------===//

// addLiveIn - This helper function adds the specified physical register to the
// MachineFunction as a live in value.  It also creates a corresponding
// virtual register for it.
static unsigned
addLiveIn(MachineFunction &MF, unsigned PReg, const TargetRegisterClass *RC)
{
  unsigned VReg = MF.getRegInfo().createVirtualRegister(RC);
  MF.getRegInfo().addLiveIn(PReg, VReg);
  return VReg;
}

static MachineBasicBlock *insertDivByZeroTrap(MachineInstr &MI,
                                              MachineBasicBlock &MBB,
                                              const TargetInstrInfo &TII,
                                              bool Is64Bit, bool IsMicroMaxis) {
  if (NoZeroDivCheck)
    return &MBB;

  // Insert instruction "teq $divisor_reg, $zero, 7".
  MachineBasicBlock::iterator I(MI);
  MachineInstrBuilder MIB;
  MachineOperand &Divisor = MI.getOperand(2);
  MIB = BuildMI(MBB, std::next(I), MI.getDebugLoc(),
                TII.get(IsMicroMaxis ? Maxis::TEQ_MM : Maxis::TEQ))
            .addReg(Divisor.getReg(), getKillRegState(Divisor.isKill()))
            .addReg(Maxis::ZERO)
            .addImm(7);

  // Use the 32-bit sub-register if this is a 64-bit division.
  if (Is64Bit)
    MIB->getOperand(0).setSubReg(Maxis::sub_32);

  // Clear Divisor's kill flag.
  Divisor.setIsKill(false);

  // We would normally delete the original instruction here but in this case
  // we only needed to inject an additional instruction rather than replace it.

  return &MBB;
}

MachineBasicBlock *
MaxisTargetLowering::EmitInstrWithCustomInserter(MachineInstr &MI,
                                                MachineBasicBlock *BB) const {
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Unexpected instr type to insert");
  case Maxis::ATOMIC_LOAD_ADD_I8:
    return emitAtomicBinaryPartword(MI, BB, 1, Maxis::ADDu);
  case Maxis::ATOMIC_LOAD_ADD_I16:
    return emitAtomicBinaryPartword(MI, BB, 2, Maxis::ADDu);
  case Maxis::ATOMIC_LOAD_ADD_I32:
    return emitAtomicBinary(MI, BB, 4, Maxis::ADDu);
  case Maxis::ATOMIC_LOAD_ADD_I64:
    return emitAtomicBinary(MI, BB, 8, Maxis::DADDu);

  case Maxis::ATOMIC_LOAD_AND_I8:
    return emitAtomicBinaryPartword(MI, BB, 1, Maxis::AND);
  case Maxis::ATOMIC_LOAD_AND_I16:
    return emitAtomicBinaryPartword(MI, BB, 2, Maxis::AND);
  case Maxis::ATOMIC_LOAD_AND_I32:
    return emitAtomicBinary(MI, BB, 4, Maxis::AND);
  case Maxis::ATOMIC_LOAD_AND_I64:
    return emitAtomicBinary(MI, BB, 8, Maxis::AND64);

  case Maxis::ATOMIC_LOAD_OR_I8:
    return emitAtomicBinaryPartword(MI, BB, 1, Maxis::OR);
  case Maxis::ATOMIC_LOAD_OR_I16:
    return emitAtomicBinaryPartword(MI, BB, 2, Maxis::OR);
  case Maxis::ATOMIC_LOAD_OR_I32:
    return emitAtomicBinary(MI, BB, 4, Maxis::OR);
  case Maxis::ATOMIC_LOAD_OR_I64:
    return emitAtomicBinary(MI, BB, 8, Maxis::OR64);

  case Maxis::ATOMIC_LOAD_XOR_I8:
    return emitAtomicBinaryPartword(MI, BB, 1, Maxis::XOR);
  case Maxis::ATOMIC_LOAD_XOR_I16:
    return emitAtomicBinaryPartword(MI, BB, 2, Maxis::XOR);
  case Maxis::ATOMIC_LOAD_XOR_I32:
    return emitAtomicBinary(MI, BB, 4, Maxis::XOR);
  case Maxis::ATOMIC_LOAD_XOR_I64:
    return emitAtomicBinary(MI, BB, 8, Maxis::XOR64);

  case Maxis::ATOMIC_LOAD_NAND_I8:
    return emitAtomicBinaryPartword(MI, BB, 1, 0, true);
  case Maxis::ATOMIC_LOAD_NAND_I16:
    return emitAtomicBinaryPartword(MI, BB, 2, 0, true);
  case Maxis::ATOMIC_LOAD_NAND_I32:
    return emitAtomicBinary(MI, BB, 4, 0, true);
  case Maxis::ATOMIC_LOAD_NAND_I64:
    return emitAtomicBinary(MI, BB, 8, 0, true);

  case Maxis::ATOMIC_LOAD_SUB_I8:
    return emitAtomicBinaryPartword(MI, BB, 1, Maxis::SUBu);
  case Maxis::ATOMIC_LOAD_SUB_I16:
    return emitAtomicBinaryPartword(MI, BB, 2, Maxis::SUBu);
  case Maxis::ATOMIC_LOAD_SUB_I32:
    return emitAtomicBinary(MI, BB, 4, Maxis::SUBu);
  case Maxis::ATOMIC_LOAD_SUB_I64:
    return emitAtomicBinary(MI, BB, 8, Maxis::DSUBu);

  case Maxis::ATOMIC_SWAP_I8:
    return emitAtomicBinaryPartword(MI, BB, 1, 0);
  case Maxis::ATOMIC_SWAP_I16:
    return emitAtomicBinaryPartword(MI, BB, 2, 0);
  case Maxis::ATOMIC_SWAP_I32:
    return emitAtomicBinary(MI, BB, 4, 0);
  case Maxis::ATOMIC_SWAP_I64:
    return emitAtomicBinary(MI, BB, 8, 0);

  case Maxis::ATOMIC_CMP_SWAP_I8:
    return emitAtomicCmpSwapPartword(MI, BB, 1);
  case Maxis::ATOMIC_CMP_SWAP_I16:
    return emitAtomicCmpSwapPartword(MI, BB, 2);
  case Maxis::ATOMIC_CMP_SWAP_I32:
    return emitAtomicCmpSwap(MI, BB, 4);
  case Maxis::ATOMIC_CMP_SWAP_I64:
    return emitAtomicCmpSwap(MI, BB, 8);
  case Maxis::PseudoSDIV:
  case Maxis::PseudoUDIV:
  case Maxis::DIV:
  case Maxis::DIVU:
  case Maxis::MOD:
  case Maxis::MODU:
    return insertDivByZeroTrap(MI, *BB, *Subtarget.getInstrInfo(), false,
                               false);
  case Maxis::SDIV_MM_Pseudo:
  case Maxis::UDIV_MM_Pseudo:
  case Maxis::SDIV_MM:
  case Maxis::UDIV_MM:
  case Maxis::DIV_MMR6:
  case Maxis::DIVU_MMR6:
  case Maxis::MOD_MMR6:
  case Maxis::MODU_MMR6:
    return insertDivByZeroTrap(MI, *BB, *Subtarget.getInstrInfo(), false, true);
  case Maxis::PseudoDSDIV:
  case Maxis::PseudoDUDIV:
  case Maxis::DDIV:
  case Maxis::DDIVU:
  case Maxis::DMOD:
  case Maxis::DMODU:
    return insertDivByZeroTrap(MI, *BB, *Subtarget.getInstrInfo(), true, false);

  case Maxis::PseudoSELECT_I:
  case Maxis::PseudoSELECT_I64:
  case Maxis::PseudoSELECT_S:
  case Maxis::PseudoSELECT_D32:
  case Maxis::PseudoSELECT_D64:
    return emitPseudoSELECT(MI, BB, false, Maxis::BNE);
  case Maxis::PseudoSELECTFP_F_I:
  case Maxis::PseudoSELECTFP_F_I64:
  case Maxis::PseudoSELECTFP_F_S:
  case Maxis::PseudoSELECTFP_F_D32:
  case Maxis::PseudoSELECTFP_F_D64:
    return emitPseudoSELECT(MI, BB, true, Maxis::BC1F);
  case Maxis::PseudoSELECTFP_T_I:
  case Maxis::PseudoSELECTFP_T_I64:
  case Maxis::PseudoSELECTFP_T_S:
  case Maxis::PseudoSELECTFP_T_D32:
  case Maxis::PseudoSELECTFP_T_D64:
    return emitPseudoSELECT(MI, BB, true, Maxis::BC1T);
  }
}

// This function also handles Maxis::ATOMIC_SWAP_I32 (when BinOpcode == 0), and
// Maxis::ATOMIC_LOAD_NAND_I32 (when Nand == true)
MachineBasicBlock *MaxisTargetLowering::emitAtomicBinary(MachineInstr &MI,
                                                        MachineBasicBlock *BB,
                                                        unsigned Size,
                                                        unsigned BinOpcode,
                                                        bool Nand) const {
  assert((Size == 4 || Size == 8) && "Unsupported size for EmitAtomicBinary.");

  MachineFunction *MF = BB->getParent();
  MachineRegisterInfo &RegInfo = MF->getRegInfo();
  const TargetRegisterClass *RC = getRegClassFor(MVT::getIntegerVT(Size * 8));
  const TargetInstrInfo *TII = Subtarget.getInstrInfo();
  const bool ArePtrs64bit = ABI.ArePtrs64bit();
  DebugLoc DL = MI.getDebugLoc();
  unsigned LL, SC, AND, NOR, ZERO, BEQ;

  if (Size == 4) {
    if (isMicroMaxis) {
      LL = Maxis::LL_MM;
      SC = Maxis::SC_MM;
    } else {
      LL = Subtarget.hasMaxis32r6()
               ? (ArePtrs64bit ? Maxis::LL64_R6 : Maxis::LL_R6)
               : (ArePtrs64bit ? Maxis::LL64 : Maxis::LL);
      SC = Subtarget.hasMaxis32r6()
               ? (ArePtrs64bit ? Maxis::SC64_R6 : Maxis::SC_R6)
               : (ArePtrs64bit ? Maxis::SC64 : Maxis::SC);
    }

    AND = Maxis::AND;
    NOR = Maxis::NOR;
    ZERO = Maxis::ZERO;
    BEQ = Maxis::BEQ;
  } else {
    LL = Subtarget.hasMaxis64r6() ? Maxis::LLD_R6 : Maxis::LLD;
    SC = Subtarget.hasMaxis64r6() ? Maxis::SCD_R6 : Maxis::SCD;
    AND = Maxis::AND64;
    NOR = Maxis::NOR64;
    ZERO = Maxis::ZERO_64;
    BEQ = Maxis::BEQ64;
  }

  unsigned OldVal = MI.getOperand(0).getReg();
  unsigned Ptr = MI.getOperand(1).getReg();
  unsigned Incr = MI.getOperand(2).getReg();

  unsigned StoreVal = RegInfo.createVirtualRegister(RC);
  unsigned AndRes = RegInfo.createVirtualRegister(RC);
  unsigned Success = RegInfo.createVirtualRegister(RC);

  // insert new blocks after the current block
  const BasicBlock *LLVM_BB = BB->getBasicBlock();
  MachineBasicBlock *loopMBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *exitMBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineFunction::iterator It = ++BB->getIterator();
  MF->insert(It, loopMBB);
  MF->insert(It, exitMBB);

  // Transfer the remainder of BB and its successor edges to exitMBB.
  exitMBB->splice(exitMBB->begin(), BB,
                  std::next(MachineBasicBlock::iterator(MI)), BB->end());
  exitMBB->transferSuccessorsAndUpdatePHIs(BB);

  //  thisMBB:
  //    ...
  //    fallthrough --> loopMBB
  BB->addSuccessor(loopMBB);
  loopMBB->addSuccessor(loopMBB);
  loopMBB->addSuccessor(exitMBB);

  //  loopMBB:
  //    ll oldval, 0(ptr)
  //    <binop> storeval, oldval, incr
  //    sc success, storeval, 0(ptr)
  //    beq success, $0, loopMBB
  BB = loopMBB;
  BuildMI(BB, DL, TII->get(LL), OldVal).addReg(Ptr).addImm(0);
  if (Nand) {
    //  and andres, oldval, incr
    //  nor storeval, $0, andres
    BuildMI(BB, DL, TII->get(AND), AndRes).addReg(OldVal).addReg(Incr);
    BuildMI(BB, DL, TII->get(NOR), StoreVal).addReg(ZERO).addReg(AndRes);
  } else if (BinOpcode) {
    //  <binop> storeval, oldval, incr
    BuildMI(BB, DL, TII->get(BinOpcode), StoreVal).addReg(OldVal).addReg(Incr);
  } else {
    StoreVal = Incr;
  }
  BuildMI(BB, DL, TII->get(SC), Success).addReg(StoreVal).addReg(Ptr).addImm(0);
  BuildMI(BB, DL, TII->get(BEQ)).addReg(Success).addReg(ZERO).addMBB(loopMBB);

  MI.eraseFromParent(); // The instruction is gone now.

  return exitMBB;
}

MachineBasicBlock *MaxisTargetLowering::emitSignExtendToI32InReg(
    MachineInstr &MI, MachineBasicBlock *BB, unsigned Size, unsigned DstReg,
    unsigned SrcReg) const {
  const TargetInstrInfo *TII = Subtarget.getInstrInfo();
  const DebugLoc &DL = MI.getDebugLoc();

  if (Subtarget.hasMaxis32r2() && Size == 1) {
    BuildMI(BB, DL, TII->get(Maxis::SEB), DstReg).addReg(SrcReg);
    return BB;
  }

  if (Subtarget.hasMaxis32r2() && Size == 2) {
    BuildMI(BB, DL, TII->get(Maxis::SEH), DstReg).addReg(SrcReg);
    return BB;
  }

  MachineFunction *MF = BB->getParent();
  MachineRegisterInfo &RegInfo = MF->getRegInfo();
  const TargetRegisterClass *RC = getRegClassFor(MVT::i32);
  unsigned ScrReg = RegInfo.createVirtualRegister(RC);

  assert(Size < 32);
  int64_t ShiftImm = 32 - (Size * 8);

  BuildMI(BB, DL, TII->get(Maxis::SLLi), ScrReg).addReg(SrcReg).addImm(ShiftImm);
  BuildMI(BB, DL, TII->get(Maxis::SRA), DstReg).addReg(ScrReg).addImm(ShiftImm);

  return BB;
}

MachineBasicBlock *MaxisTargetLowering::emitAtomicBinaryPartword(
    MachineInstr &MI, MachineBasicBlock *BB, unsigned Size, unsigned BinOpcode,
    bool Nand) const {
  assert((Size == 1 || Size == 2) &&
         "Unsupported size for EmitAtomicBinaryPartial.");

  MachineFunction *MF = BB->getParent();
  MachineRegisterInfo &RegInfo = MF->getRegInfo();
  const TargetRegisterClass *RC = getRegClassFor(MVT::i32);
  const bool ArePtrs64bit = ABI.ArePtrs64bit();
  const TargetRegisterClass *RCp =
    getRegClassFor(ArePtrs64bit ? MVT::i64 : MVT::i32);
  const TargetInstrInfo *TII = Subtarget.getInstrInfo();
  DebugLoc DL = MI.getDebugLoc();

  unsigned Dest = MI.getOperand(0).getReg();
  unsigned Ptr = MI.getOperand(1).getReg();
  unsigned Incr = MI.getOperand(2).getReg();

  unsigned AlignedAddr = RegInfo.createVirtualRegister(RCp);
  unsigned ShiftAmt = RegInfo.createVirtualRegister(RC);
  unsigned Mask = RegInfo.createVirtualRegister(RC);
  unsigned Mask2 = RegInfo.createVirtualRegister(RC);
  unsigned NewVal = RegInfo.createVirtualRegister(RC);
  unsigned OldVal = RegInfo.createVirtualRegister(RC);
  unsigned Incr2 = RegInfo.createVirtualRegister(RC);
  unsigned MaskLSB2 = RegInfo.createVirtualRegister(RCp);
  unsigned PtrLSB2 = RegInfo.createVirtualRegister(RC);
  unsigned MaskUpper = RegInfo.createVirtualRegister(RC);
  unsigned AndRes = RegInfo.createVirtualRegister(RC);
  unsigned BinOpRes = RegInfo.createVirtualRegister(RC);
  unsigned MaskedOldVal0 = RegInfo.createVirtualRegister(RC);
  unsigned StoreVal = RegInfo.createVirtualRegister(RC);
  unsigned MaskedOldVal1 = RegInfo.createVirtualRegister(RC);
  unsigned SrlRes = RegInfo.createVirtualRegister(RC);
  unsigned Success = RegInfo.createVirtualRegister(RC);

  unsigned LL, SC;
  if (isMicroMaxis) {
    LL = Maxis::LL_MM;
    SC = Maxis::SC_MM;
  } else {
    LL = Subtarget.hasMaxis32r6() ? (ArePtrs64bit ? Maxis::LL64_R6 : Maxis::LL_R6)
                                 : (ArePtrs64bit ? Maxis::LL64 : Maxis::LL);
    SC = Subtarget.hasMaxis32r6() ? (ArePtrs64bit ? Maxis::SC64_R6 : Maxis::SC_R6)
                                 : (ArePtrs64bit ? Maxis::SC64 : Maxis::SC);
  }

  // insert new blocks after the current block
  const BasicBlock *LLVM_BB = BB->getBasicBlock();
  MachineBasicBlock *loopMBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *sinkMBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *exitMBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineFunction::iterator It = ++BB->getIterator();
  MF->insert(It, loopMBB);
  MF->insert(It, sinkMBB);
  MF->insert(It, exitMBB);

  // Transfer the remainder of BB and its successor edges to exitMBB.
  exitMBB->splice(exitMBB->begin(), BB,
                  std::next(MachineBasicBlock::iterator(MI)), BB->end());
  exitMBB->transferSuccessorsAndUpdatePHIs(BB);

  BB->addSuccessor(loopMBB);
  loopMBB->addSuccessor(loopMBB);
  loopMBB->addSuccessor(sinkMBB);
  sinkMBB->addSuccessor(exitMBB);

  //  thisMBB:
  //    addi   masklsb2,$0,-4                # 0xfffffffc
  //    and     alignedaddr,ptr,masklsb2
  //    andi    ptrlsb2,ptr,3
  //    slli     shiftamt,ptrlsb2,3
  //    ori     maskupper,$0,255               # 0xff
  //    slli     mask,maskupper,shiftamt
  //    nor     mask2,$0,mask
  //    slli     incr2,incr,shiftamt

  int64_t MaskImm = (Size == 1) ? 255 : 65535;
  BuildMI(BB, DL, TII->get(ABI.GetPtrAddiOp()), MaskLSB2)
    .addReg(ABI.GetNullPtr()).addImm(-4);
  BuildMI(BB, DL, TII->get(ABI.GetPtrAndOp()), AlignedAddr)
    .addReg(Ptr).addReg(MaskLSB2);
  BuildMI(BB, DL, TII->get(Maxis::ANDi), PtrLSB2)
      .addReg(Ptr, 0, ArePtrs64bit ? Maxis::sub_32 : 0).addImm(3);
  if (Subtarget.isLittle()) {
    BuildMI(BB, DL, TII->get(Maxis::SLLi), ShiftAmt).addReg(PtrLSB2).addImm(3);
  } else {
    unsigned Off = RegInfo.createVirtualRegister(RC);
    BuildMI(BB, DL, TII->get(Maxis::XORi), Off)
      .addReg(PtrLSB2).addImm((Size == 1) ? 3 : 2);
    BuildMI(BB, DL, TII->get(Maxis::SLLi), ShiftAmt).addReg(Off).addImm(3);
  }
  BuildMI(BB, DL, TII->get(Maxis::ORi), MaskUpper)
    .addReg(Maxis::ZERO).addImm(MaskImm);
  BuildMI(BB, DL, TII->get(Maxis::SLL), Mask)
    .addReg(MaskUpper).addReg(ShiftAmt);
  BuildMI(BB, DL, TII->get(Maxis::NOR), Mask2).addReg(Maxis::ZERO).addReg(Mask);
  BuildMI(BB, DL, TII->get(Maxis::SLL), Incr2).addReg(Incr).addReg(ShiftAmt);

  // atomic.load.binop
  // loopMBB:
  //   ll      oldval,0(alignedaddr)
  //   binop   binopres,oldval,incr2
  //   and     newval,binopres,mask
  //   and     maskedoldval0,oldval,mask2
  //   or      storeval,maskedoldval0,newval
  //   sc      success,storeval,0(alignedaddr)
  //   beq     success,$0,loopMBB

  // atomic.swap
  // loopMBB:
  //   ll      oldval,0(alignedaddr)
  //   and     newval,incr2,mask
  //   and     maskedoldval0,oldval,mask2
  //   or      storeval,maskedoldval0,newval
  //   sc      success,storeval,0(alignedaddr)
  //   beq     success,$0,loopMBB

  BB = loopMBB;
  BuildMI(BB, DL, TII->get(LL), OldVal).addReg(AlignedAddr).addImm(0);
  if (Nand) {
    //  and andres, oldval, incr2
    //  nor binopres, $0, andres
    //  and newval, binopres, mask
    BuildMI(BB, DL, TII->get(Maxis::AND), AndRes).addReg(OldVal).addReg(Incr2);
    BuildMI(BB, DL, TII->get(Maxis::NOR), BinOpRes)
      .addReg(Maxis::ZERO).addReg(AndRes);
    BuildMI(BB, DL, TII->get(Maxis::AND), NewVal).addReg(BinOpRes).addReg(Mask);
  } else if (BinOpcode) {
    //  <binop> binopres, oldval, incr2
    //  and newval, binopres, mask
    BuildMI(BB, DL, TII->get(BinOpcode), BinOpRes).addReg(OldVal).addReg(Incr2);
    BuildMI(BB, DL, TII->get(Maxis::AND), NewVal).addReg(BinOpRes).addReg(Mask);
  } else { // atomic.swap
    //  and newval, incr2, mask
    BuildMI(BB, DL, TII->get(Maxis::AND), NewVal).addReg(Incr2).addReg(Mask);
  }

  BuildMI(BB, DL, TII->get(Maxis::AND), MaskedOldVal0)
    .addReg(OldVal).addReg(Mask2);
  BuildMI(BB, DL, TII->get(Maxis::OR), StoreVal)
    .addReg(MaskedOldVal0).addReg(NewVal);
  BuildMI(BB, DL, TII->get(SC), Success)
    .addReg(StoreVal).addReg(AlignedAddr).addImm(0);
  BuildMI(BB, DL, TII->get(Maxis::BEQ))
    .addReg(Success).addReg(Maxis::ZERO).addMBB(loopMBB);

  //  sinkMBB:
  //    and     maskedoldval1,oldval,mask
  //    srl     srlres,maskedoldval1,shiftamt
  //    sign_extend dest,srlres
  BB = sinkMBB;

  BuildMI(BB, DL, TII->get(Maxis::AND), MaskedOldVal1)
    .addReg(OldVal).addReg(Mask);
  BuildMI(BB, DL, TII->get(Maxis::SRLV), SrlRes)
      .addReg(MaskedOldVal1).addReg(ShiftAmt);
  BB = emitSignExtendToI32InReg(MI, BB, Size, Dest, SrlRes);

  MI.eraseFromParent(); // The instruction is gone now.

  return exitMBB;
}

MachineBasicBlock *MaxisTargetLowering::emitAtomicCmpSwap(MachineInstr &MI,
                                                         MachineBasicBlock *BB,
                                                         unsigned Size) const {
  assert((Size == 4 || Size == 8) && "Unsupported size for EmitAtomicCmpSwap.");

  MachineFunction *MF = BB->getParent();
  MachineRegisterInfo &RegInfo = MF->getRegInfo();
  const TargetRegisterClass *RC = getRegClassFor(MVT::getIntegerVT(Size * 8));
  const TargetInstrInfo *TII = Subtarget.getInstrInfo();
  const bool ArePtrs64bit = ABI.ArePtrs64bit();
  DebugLoc DL = MI.getDebugLoc();
  unsigned LL, SC, ZERO, BNE, BEQ;

  if (Size == 4) {
    if (isMicroMaxis) {
      LL = Maxis::LL_MM;
      SC = Maxis::SC_MM;
    } else {
      LL = Subtarget.hasMaxis32r6()
               ? (ArePtrs64bit ? Maxis::LL64_R6 : Maxis::LL_R6)
               : (ArePtrs64bit ? Maxis::LL64 : Maxis::LL);
      SC = Subtarget.hasMaxis32r6()
               ? (ArePtrs64bit ? Maxis::SC64_R6 : Maxis::SC_R6)
               : (ArePtrs64bit ? Maxis::SC64 : Maxis::SC);
    }

    ZERO = Maxis::ZERO;
    BNE = Maxis::BNE;
    BEQ = Maxis::BEQ;
  } else {
    LL = Subtarget.hasMaxis64r6() ? Maxis::LLD_R6 : Maxis::LLD;
    SC = Subtarget.hasMaxis64r6() ? Maxis::SCD_R6 : Maxis::SCD;
    ZERO = Maxis::ZERO_64;
    BNE = Maxis::BNE64;
    BEQ = Maxis::BEQ64;
  }

  unsigned Dest = MI.getOperand(0).getReg();
  unsigned Ptr = MI.getOperand(1).getReg();
  unsigned OldVal = MI.getOperand(2).getReg();
  unsigned NewVal = MI.getOperand(3).getReg();

  unsigned Success = RegInfo.createVirtualRegister(RC);

  // insert new blocks after the current block
  const BasicBlock *LLVM_BB = BB->getBasicBlock();
  MachineBasicBlock *loop1MBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *loop2MBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *exitMBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineFunction::iterator It = ++BB->getIterator();
  MF->insert(It, loop1MBB);
  MF->insert(It, loop2MBB);
  MF->insert(It, exitMBB);

  // Transfer the remainder of BB and its successor edges to exitMBB.
  exitMBB->splice(exitMBB->begin(), BB,
                  std::next(MachineBasicBlock::iterator(MI)), BB->end());
  exitMBB->transferSuccessorsAndUpdatePHIs(BB);

  //  thisMBB:
  //    ...
  //    fallthrough --> loop1MBB
  BB->addSuccessor(loop1MBB);
  loop1MBB->addSuccessor(exitMBB);
  loop1MBB->addSuccessor(loop2MBB);
  loop2MBB->addSuccessor(loop1MBB);
  loop2MBB->addSuccessor(exitMBB);

  // loop1MBB:
  //   ll dest, 0(ptr)
  //   bne dest, oldval, exitMBB
  BB = loop1MBB;
  BuildMI(BB, DL, TII->get(LL), Dest).addReg(Ptr).addImm(0);
  BuildMI(BB, DL, TII->get(BNE))
    .addReg(Dest).addReg(OldVal).addMBB(exitMBB);

  // loop2MBB:
  //   sc success, newval, 0(ptr)
  //   beq success, $0, loop1MBB
  BB = loop2MBB;
  BuildMI(BB, DL, TII->get(SC), Success)
    .addReg(NewVal).addReg(Ptr).addImm(0);
  BuildMI(BB, DL, TII->get(BEQ))
    .addReg(Success).addReg(ZERO).addMBB(loop1MBB);

  MI.eraseFromParent(); // The instruction is gone now.

  return exitMBB;
}

MachineBasicBlock *MaxisTargetLowering::emitAtomicCmpSwapPartword(
    MachineInstr &MI, MachineBasicBlock *BB, unsigned Size) const {
  assert((Size == 1 || Size == 2) &&
      "Unsupported size for EmitAtomicCmpSwapPartial.");

  MachineFunction *MF = BB->getParent();
  MachineRegisterInfo &RegInfo = MF->getRegInfo();
  const TargetRegisterClass *RC = getRegClassFor(MVT::i32);
  const bool ArePtrs64bit = ABI.ArePtrs64bit();
  const TargetRegisterClass *RCp =
    getRegClassFor(ArePtrs64bit ? MVT::i64 : MVT::i32);
  const TargetInstrInfo *TII = Subtarget.getInstrInfo();
  DebugLoc DL = MI.getDebugLoc();

  unsigned Dest = MI.getOperand(0).getReg();
  unsigned Ptr = MI.getOperand(1).getReg();
  unsigned CmpVal = MI.getOperand(2).getReg();
  unsigned NewVal = MI.getOperand(3).getReg();

  unsigned AlignedAddr = RegInfo.createVirtualRegister(RCp);
  unsigned ShiftAmt = RegInfo.createVirtualRegister(RC);
  unsigned Mask = RegInfo.createVirtualRegister(RC);
  unsigned Mask2 = RegInfo.createVirtualRegister(RC);
  unsigned ShiftedCmpVal = RegInfo.createVirtualRegister(RC);
  unsigned OldVal = RegInfo.createVirtualRegister(RC);
  unsigned MaskedOldVal0 = RegInfo.createVirtualRegister(RC);
  unsigned ShiftedNewVal = RegInfo.createVirtualRegister(RC);
  unsigned MaskLSB2 = RegInfo.createVirtualRegister(RCp);
  unsigned PtrLSB2 = RegInfo.createVirtualRegister(RC);
  unsigned MaskUpper = RegInfo.createVirtualRegister(RC);
  unsigned MaskedCmpVal = RegInfo.createVirtualRegister(RC);
  unsigned MaskedNewVal = RegInfo.createVirtualRegister(RC);
  unsigned MaskedOldVal1 = RegInfo.createVirtualRegister(RC);
  unsigned StoreVal = RegInfo.createVirtualRegister(RC);
  unsigned SrlRes = RegInfo.createVirtualRegister(RC);
  unsigned Success = RegInfo.createVirtualRegister(RC);
  unsigned LL, SC;

  if (isMicroMaxis) {
    LL = Maxis::LL_MM;
    SC = Maxis::SC_MM;
  } else {
    LL = Subtarget.hasMaxis32r6() ? (ArePtrs64bit ? Maxis::LL64_R6 : Maxis::LL_R6)
                                 : (ArePtrs64bit ? Maxis::LL64 : Maxis::LL);
    SC = Subtarget.hasMaxis32r6() ? (ArePtrs64bit ? Maxis::SC64_R6 : Maxis::SC_R6)
                                 : (ArePtrs64bit ? Maxis::SC64 : Maxis::SC);
  }

  // insert new blocks after the current block
  const BasicBlock *LLVM_BB = BB->getBasicBlock();
  MachineBasicBlock *loop1MBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *loop2MBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *sinkMBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *exitMBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineFunction::iterator It = ++BB->getIterator();
  MF->insert(It, loop1MBB);
  MF->insert(It, loop2MBB);
  MF->insert(It, sinkMBB);
  MF->insert(It, exitMBB);

  // Transfer the remainder of BB and its successor edges to exitMBB.
  exitMBB->splice(exitMBB->begin(), BB,
                  std::next(MachineBasicBlock::iterator(MI)), BB->end());
  exitMBB->transferSuccessorsAndUpdatePHIs(BB);

  BB->addSuccessor(loop1MBB);
  loop1MBB->addSuccessor(sinkMBB);
  loop1MBB->addSuccessor(loop2MBB);
  loop2MBB->addSuccessor(loop1MBB);
  loop2MBB->addSuccessor(sinkMBB);
  sinkMBB->addSuccessor(exitMBB);

  // FIXME: computation of newval2 can be moved to loop2MBB.
  //  thisMBB:
  //    addi   masklsb2,$0,-4                # 0xfffffffc
  //    and     alignedaddr,ptr,masklsb2
  //    andi    ptrlsb2,ptr,3
  //    xori    ptrlsb2,ptrlsb2,3              # Only for BE
  //    slli     shiftamt,ptrlsb2,3
  //    ori     maskupper,$0,255               # 0xff
  //    slli     mask,maskupper,shiftamt
  //    nor     mask2,$0,mask
  //    andi    maskedcmpval,cmpval,255
  //    slli     shiftedcmpval,maskedcmpval,shiftamt
  //    andi    maskednewval,newval,255
  //    slli     shiftednewval,maskednewval,shiftamt
  int64_t MaskImm = (Size == 1) ? 255 : 65535;
  BuildMI(BB, DL, TII->get(ArePtrs64bit ? Maxis::DADDi : Maxis::ADDi), MaskLSB2)
    .addReg(ABI.GetNullPtr()).addImm(-4);
  BuildMI(BB, DL, TII->get(ArePtrs64bit ? Maxis::AND64 : Maxis::AND), AlignedAddr)
    .addReg(Ptr).addReg(MaskLSB2);
  BuildMI(BB, DL, TII->get(Maxis::ANDi), PtrLSB2)
      .addReg(Ptr, 0, ArePtrs64bit ? Maxis::sub_32 : 0).addImm(3);
  if (Subtarget.isLittle()) {
    BuildMI(BB, DL, TII->get(Maxis::SLLi), ShiftAmt).addReg(PtrLSB2).addImm(3);
  } else {
    unsigned Off = RegInfo.createVirtualRegister(RC);
    BuildMI(BB, DL, TII->get(Maxis::XORi), Off)
      .addReg(PtrLSB2).addImm((Size == 1) ? 3 : 2);
    BuildMI(BB, DL, TII->get(Maxis::SLLi), ShiftAmt).addReg(Off).addImm(3);
  }
  BuildMI(BB, DL, TII->get(Maxis::ORi), MaskUpper)
    .addReg(Maxis::ZERO).addImm(MaskImm);
  BuildMI(BB, DL, TII->get(Maxis::SLL), Mask)
    .addReg(MaskUpper).addReg(ShiftAmt);
  BuildMI(BB, DL, TII->get(Maxis::NOR), Mask2).addReg(Maxis::ZERO).addReg(Mask);
  BuildMI(BB, DL, TII->get(Maxis::ANDi), MaskedCmpVal)
    .addReg(CmpVal).addImm(MaskImm);
  BuildMI(BB, DL, TII->get(Maxis::SLL), ShiftedCmpVal)
    .addReg(MaskedCmpVal).addReg(ShiftAmt);
  BuildMI(BB, DL, TII->get(Maxis::ANDi), MaskedNewVal)
    .addReg(NewVal).addImm(MaskImm);
  BuildMI(BB, DL, TII->get(Maxis::SLL), ShiftedNewVal)
    .addReg(MaskedNewVal).addReg(ShiftAmt);

  //  loop1MBB:
  //    ll      oldval,0(alginedaddr)
  //    and     maskedoldval0,oldval,mask
  //    bne     maskedoldval0,shiftedcmpval,sinkMBB
  BB = loop1MBB;
  BuildMI(BB, DL, TII->get(LL), OldVal).addReg(AlignedAddr).addImm(0);
  BuildMI(BB, DL, TII->get(Maxis::AND), MaskedOldVal0)
    .addReg(OldVal).addReg(Mask);
  BuildMI(BB, DL, TII->get(Maxis::BNE))
    .addReg(MaskedOldVal0).addReg(ShiftedCmpVal).addMBB(sinkMBB);

  //  loop2MBB:
  //    and     maskedoldval1,oldval,mask2
  //    or      storeval,maskedoldval1,shiftednewval
  //    sc      success,storeval,0(alignedaddr)
  //    beq     success,$0,loop1MBB
  BB = loop2MBB;
  BuildMI(BB, DL, TII->get(Maxis::AND), MaskedOldVal1)
    .addReg(OldVal).addReg(Mask2);
  BuildMI(BB, DL, TII->get(Maxis::OR), StoreVal)
    .addReg(MaskedOldVal1).addReg(ShiftedNewVal);
  BuildMI(BB, DL, TII->get(SC), Success)
      .addReg(StoreVal).addReg(AlignedAddr).addImm(0);
  BuildMI(BB, DL, TII->get(Maxis::BEQ))
      .addReg(Success).addReg(Maxis::ZERO).addMBB(loop1MBB);

  //  sinkMBB:
  //    srl     srlres,maskedoldval0,shiftamt
  //    sign_extend dest,srlres
  BB = sinkMBB;

  BuildMI(BB, DL, TII->get(Maxis::SRLV), SrlRes)
      .addReg(MaskedOldVal0).addReg(ShiftAmt);
  BB = emitSignExtendToI32InReg(MI, BB, Size, Dest, SrlRes);

  MI.eraseFromParent(); // The instruction is gone now.

  return exitMBB;
}

SDValue MaxisTargetLowering::lowerBRCOND(SDValue Op, SelectionDAG &DAG) const {
  // The first operand is the chain, the second is the condition, the third is
  // the block to branch to if the condition is true.
  SDValue Chain = Op.getOperand(0);
  SDValue Dest = Op.getOperand(2);
  SDLoc DL(Op);

  assert(!Subtarget.hasMaxis32r6() && !Subtarget.hasMaxis64r6());
  SDValue CondRes = createFPCmp(DAG, Op.getOperand(1));

  // Return if flag is not set by a floating point comparison.
  if (CondRes.getOpcode() != MaxisISD::FPCmp)
    return Op;

  SDValue CCNode  = CondRes.getOperand(2);
  Maxis::CondCode CC =
    (Maxis::CondCode)cast<ConstantSDNode>(CCNode)->getZExtValue();
  unsigned Opc = invertFPCondCodeUser(CC) ? Maxis::BRANCH_F : Maxis::BRANCH_T;
  SDValue BrCode = DAG.getConstant(Opc, DL, MVT::i32);
  SDValue FCC0 = DAG.getRegister(Maxis::FCC0, MVT::i32);
  return DAG.getNode(MaxisISD::FPBrcond, DL, Op.getValueType(), Chain, BrCode,
                     FCC0, Dest, CondRes);
}

SDValue MaxisTargetLowering::
lowerSELECT(SDValue Op, SelectionDAG &DAG) const
{
  assert(!Subtarget.hasMaxis32r6() && !Subtarget.hasMaxis64r6());
  SDValue Cond = createFPCmp(DAG, Op.getOperand(0));

  // Return if flag is not set by a floating point comparison.
  if (Cond.getOpcode() != MaxisISD::FPCmp)
    return Op;

  return createCMovFP(DAG, Cond, Op.getOperand(1), Op.getOperand(2),
                      SDLoc(Op));
}

SDValue MaxisTargetLowering::lowerSETCC(SDValue Op, SelectionDAG &DAG) const {
  assert(!Subtarget.hasMaxis32r6() && !Subtarget.hasMaxis64r6());
  SDValue Cond = createFPCmp(DAG, Op);

  assert(Cond.getOpcode() == MaxisISD::FPCmp &&
         "Floating point operand expected.");

  SDLoc DL(Op);
  SDValue True  = DAG.getConstant(1, DL, MVT::i32);
  SDValue False = DAG.getConstant(0, DL, MVT::i32);

  return createCMovFP(DAG, Cond, True, False, DL);
}

SDValue MaxisTargetLowering::lowerGlobalAddress(SDValue Op,
                                               SelectionDAG &DAG) const {
  EVT Ty = Op.getValueType();
  GlobalAddressSDNode *N = cast<GlobalAddressSDNode>(Op);
  const GlobalValue *GV = N->getGlobal();

  if (!isPositionIndependent()) {
    const MaxisTargetObjectFile *TLOF =
        static_cast<const MaxisTargetObjectFile *>(
            getTargetMachine().getObjFileLowering());
    const GlobalObject *GO = GV->getBaseObject();
    if (GO && TLOF->IsGlobalInSmallSection(GO, getTargetMachine()))
      // %gp_rel relocation
      return getAddrGPRel(N, SDLoc(N), Ty, DAG, ABI.IsN64());

                                 // %hi/%lo relocation
    return Subtarget.hasSym32() ? getAddrNonPIC(N, SDLoc(N), Ty, DAG)
                                 // %highest/%higher/%hi/%lo relocation
                                 : getAddrNonPICSym64(N, SDLoc(N), Ty, DAG);
  }

  // Every other architecture would use shouldAssumeDSOLocal in here, but
  // maxis is special.
  // * In PIC code maxis requires got loads even for local statics!
  // * To save on got entries, for local statics the got entry contains the
  //   page and an additional add instruction takes care of the low bits.
  // * It is legal to access a hidden symbol with a non hidden undefined,
  //   so one cannot guarantee that all access to a hidden symbol will know
  //   it is hidden.
  // * Maxis linkers don't support creating a page and a full got entry for
  //   the same symbol.
  // * Given all that, we have to use a full got entry for hidden symbols :-(
  if (GV->hasLocalLinkage())
    return getAddrLocal(N, SDLoc(N), Ty, DAG, ABI.IsN32() || ABI.IsN64());

  if (LargeGOT)
    return getAddrGlobalLargeGOT(
        N, SDLoc(N), Ty, DAG, MaxisII::MO_GOT_HI16, MaxisII::MO_GOT_LO16,
        DAG.getEntryNode(),
        MachinePointerInfo::getGOT(DAG.getMachineFunction()));

  return getAddrGlobal(
      N, SDLoc(N), Ty, DAG,
      (ABI.IsN32() || ABI.IsN64()) ? MaxisII::MO_GOT_DISP : MaxisII::MO_GOT,
      DAG.getEntryNode(), MachinePointerInfo::getGOT(DAG.getMachineFunction()));
}

SDValue MaxisTargetLowering::lowerBlockAddress(SDValue Op,
                                              SelectionDAG &DAG) const {
  BlockAddressSDNode *N = cast<BlockAddressSDNode>(Op);
  EVT Ty = Op.getValueType();

  if (!isPositionIndependent())
    return Subtarget.hasSym32() ? getAddrNonPIC(N, SDLoc(N), Ty, DAG)
                                : getAddrNonPICSym64(N, SDLoc(N), Ty, DAG);

  return getAddrLocal(N, SDLoc(N), Ty, DAG, ABI.IsN32() || ABI.IsN64());
}

SDValue MaxisTargetLowering::
lowerGlobalTLSAddress(SDValue Op, SelectionDAG &DAG) const
{
  // If the relocation model is PIC, use the General Dynamic TLS Model or
  // Local Dynamic TLS model, otherwise use the Initial Exec or
  // Local Exec TLS Model.

  GlobalAddressSDNode *GA = cast<GlobalAddressSDNode>(Op);
  if (DAG.getTarget().Options.EmulatedTLS)
    return LowerToTLSEmulatedModel(GA, DAG);

  SDLoc DL(GA);
  const GlobalValue *GV = GA->getGlobal();
  EVT PtrVT = getPointerTy(DAG.getDataLayout());

  TLSModel::Model model = getTargetMachine().getTLSModel(GV);

  if (model == TLSModel::GeneralDynamic || model == TLSModel::LocalDynamic) {
    // General Dynamic and Local Dynamic TLS Model.
    unsigned Flag = (model == TLSModel::LocalDynamic) ? MaxisII::MO_TLSLDM
                                                      : MaxisII::MO_TLSGD;

    SDValue TGA = DAG.getTargetGlobalAddress(GV, DL, PtrVT, 0, Flag);
    SDValue Argument = DAG.getNode(MaxisISD::Wrapper, DL, PtrVT,
                                   getGlobalReg(DAG, PtrVT), TGA);
    unsigned PtrSize = PtrVT.getSizeInBits();
    IntegerType *PtrTy = Type::getIntNTy(*DAG.getContext(), PtrSize);

    SDValue TlsGetAddr = DAG.getExternalSymbol("__tls_get_addr", PtrVT);

    ArgListTy Args;
    ArgListEntry Entry;
    Entry.Node = Argument;
    Entry.Ty = PtrTy;
    Args.push_back(Entry);

    TargetLowering::CallLoweringInfo CLI(DAG);
    CLI.setDebugLoc(DL)
        .setChain(DAG.getEntryNode())
        .setLibCallee(CallingConv::C, PtrTy, TlsGetAddr, std::move(Args));
    std::pair<SDValue, SDValue> CallResult = LowerCallTo(CLI);

    SDValue Ret = CallResult.first;

    if (model != TLSModel::LocalDynamic)
      return Ret;

    SDValue TGAHi = DAG.getTargetGlobalAddress(GV, DL, PtrVT, 0,
                                               MaxisII::MO_DTPREL_HI);
    SDValue Hi = DAG.getNode(MaxisISD::Hi, DL, PtrVT, TGAHi);
    SDValue TGALo = DAG.getTargetGlobalAddress(GV, DL, PtrVT, 0,
                                               MaxisII::MO_DTPREL_LO);
    SDValue Lo = DAG.getNode(MaxisISD::Lo, DL, PtrVT, TGALo);
    SDValue Add = DAG.getNode(ISD::ADD, DL, PtrVT, Hi, Ret);
    return DAG.getNode(ISD::ADD, DL, PtrVT, Add, Lo);
  }

  SDValue Offset;
  if (model == TLSModel::InitialExec) {
    // Initial Exec TLS Model
    SDValue TGA = DAG.getTargetGlobalAddress(GV, DL, PtrVT, 0,
                                             MaxisII::MO_GOTTPREL);
    TGA = DAG.getNode(MaxisISD::Wrapper, DL, PtrVT, getGlobalReg(DAG, PtrVT),
                      TGA);
    Offset =
        DAG.getLoad(PtrVT, DL, DAG.getEntryNode(), TGA, MachinePointerInfo());
  } else {
    // Local Exec TLS Model
    assert(model == TLSModel::LocalExec);
    SDValue TGAHi = DAG.getTargetGlobalAddress(GV, DL, PtrVT, 0,
                                               MaxisII::MO_TPREL_HI);
    SDValue TGALo = DAG.getTargetGlobalAddress(GV, DL, PtrVT, 0,
                                               MaxisII::MO_TPREL_LO);
    SDValue Hi = DAG.getNode(MaxisISD::Hi, DL, PtrVT, TGAHi);
    SDValue Lo = DAG.getNode(MaxisISD::Lo, DL, PtrVT, TGALo);
    Offset = DAG.getNode(ISD::ADD, DL, PtrVT, Hi, Lo);
  }

  SDValue ThreadPointer = DAG.getNode(MaxisISD::ThreadPointer, DL, PtrVT);
  return DAG.getNode(ISD::ADD, DL, PtrVT, ThreadPointer, Offset);
}

SDValue MaxisTargetLowering::
lowerJumpTable(SDValue Op, SelectionDAG &DAG) const
{
  JumpTableSDNode *N = cast<JumpTableSDNode>(Op);
  EVT Ty = Op.getValueType();

  if (!isPositionIndependent())
    return Subtarget.hasSym32() ? getAddrNonPIC(N, SDLoc(N), Ty, DAG)
                                : getAddrNonPICSym64(N, SDLoc(N), Ty, DAG);

  return getAddrLocal(N, SDLoc(N), Ty, DAG, ABI.IsN32() || ABI.IsN64());
}

SDValue MaxisTargetLowering::
lowerConstantPool(SDValue Op, SelectionDAG &DAG) const
{
  ConstantPoolSDNode *N = cast<ConstantPoolSDNode>(Op);
  EVT Ty = Op.getValueType();

  if (!isPositionIndependent()) {
    const MaxisTargetObjectFile *TLOF =
        static_cast<const MaxisTargetObjectFile *>(
            getTargetMachine().getObjFileLowering());

    if (TLOF->IsConstantInSmallSection(DAG.getDataLayout(), N->getConstVal(),
                                       getTargetMachine()))
      // %gp_rel relocation
      return getAddrGPRel(N, SDLoc(N), Ty, DAG, ABI.IsN64());

    return Subtarget.hasSym32() ? getAddrNonPIC(N, SDLoc(N), Ty, DAG)
                                : getAddrNonPICSym64(N, SDLoc(N), Ty, DAG);
  }

 return getAddrLocal(N, SDLoc(N), Ty, DAG, ABI.IsN32() || ABI.IsN64());
}

SDValue MaxisTargetLowering::lowerVASTART(SDValue Op, SelectionDAG &DAG) const {
  MachineFunction &MF = DAG.getMachineFunction();
  MaxisFunctionInfo *FuncInfo = MF.getInfo<MaxisFunctionInfo>();

  SDLoc DL(Op);
  SDValue FI = DAG.getFrameIndex(FuncInfo->getVarArgsFrameIndex(),
                                 getPointerTy(MF.getDataLayout()));

  // vastart just stores the address of the VarArgsFrameIndex slot into the
  // memory location argument.
  const Value *SV = cast<SrcValueSDNode>(Op.getOperand(2))->getValue();
  return DAG.getStore(Op.getOperand(0), DL, FI, Op.getOperand(1),
                      MachinePointerInfo(SV));
}

SDValue MaxisTargetLowering::lowerVAARG(SDValue Op, SelectionDAG &DAG) const {
  SDNode *Node = Op.getNode();
  EVT VT = Node->getValueType(0);
  SDValue Chain = Node->getOperand(0);
  SDValue VAListPtr = Node->getOperand(1);
  unsigned Align = Node->getConstantOperandVal(3);
  const Value *SV = cast<SrcValueSDNode>(Node->getOperand(2))->getValue();
  SDLoc DL(Node);
  unsigned ArgSlotSizeInBytes = (ABI.IsN32() || ABI.IsN64()) ? 8 : 4;

  SDValue VAListLoad = DAG.getLoad(getPointerTy(DAG.getDataLayout()), DL, Chain,
                                   VAListPtr, MachinePointerInfo(SV));
  SDValue VAList = VAListLoad;

  // Re-align the pointer if necessary.
  // It should only ever be necessary for 64-bit types on O32 since the minimum
  // argument alignment is the same as the maximum type alignment for N32/N64.
  //
  // FIXME: We currently align too often. The code generator doesn't notice
  //        when the pointer is still aligned from the last va_arg (or pair of
  //        va_args for the i64 on O32 case).
  if (Align > getMinStackArgumentAlignment()) {
    assert(((Align & (Align-1)) == 0) && "Expected Align to be a power of 2");

    VAList = DAG.getNode(ISD::ADD, DL, VAList.getValueType(), VAList,
                         DAG.getConstant(Align - 1, DL, VAList.getValueType()));

    VAList = DAG.getNode(ISD::AND, DL, VAList.getValueType(), VAList,
                         DAG.getConstant(-(int64_t)Align, DL,
                                         VAList.getValueType()));
  }

  // Increment the pointer, VAList, to the next vaarg.
  auto &TD = DAG.getDataLayout();
  unsigned ArgSizeInBytes =
      TD.getTypeAllocSize(VT.getTypeForEVT(*DAG.getContext()));
  SDValue Tmp3 =
      DAG.getNode(ISD::ADD, DL, VAList.getValueType(), VAList,
                  DAG.getConstant(alignTo(ArgSizeInBytes, ArgSlotSizeInBytes),
                                  DL, VAList.getValueType()));
  // Store the incremented VAList to the legalized pointer
  Chain = DAG.getStore(VAListLoad.getValue(1), DL, Tmp3, VAListPtr,
                       MachinePointerInfo(SV));

  // In big-endian mode we must adjust the pointer when the load size is smaller
  // than the argument slot size. We must also reduce the known alignment to
  // match. For example in the N64 ABI, we must add 4 bytes to the offset to get
  // the correct half of the slot, and reduce the alignment from 8 (slot
  // alignment) down to 4 (type alignment).
  if (!Subtarget.isLittle() && ArgSizeInBytes < ArgSlotSizeInBytes) {
    unsigned Adjustment = ArgSlotSizeInBytes - ArgSizeInBytes;
    VAList = DAG.getNode(ISD::ADD, DL, VAListPtr.getValueType(), VAList,
                         DAG.getIntPtrConstant(Adjustment, DL));
  }
  // Load the actual argument out of the pointer VAList
  return DAG.getLoad(VT, DL, Chain, VAList, MachinePointerInfo());
}

static SDValue lowerFCOPYSIGN32(SDValue Op, SelectionDAG &DAG,
                                bool HasExtractInsert) {
  EVT TyX = Op.getOperand(0).getValueType();
  EVT TyY = Op.getOperand(1).getValueType();
  SDLoc DL(Op);
  SDValue Const1 = DAG.getConstant(1, DL, MVT::i32);
  SDValue Const31 = DAG.getConstant(31, DL, MVT::i32);
  SDValue Res;

  // If operand is of type f64, extract the upper 32-bit. Otherwise, bitcast it
  // to i32.
  SDValue X = (TyX == MVT::f32) ?
    DAG.getNode(ISD::BITCAST, DL, MVT::i32, Op.getOperand(0)) :
    DAG.getNode(MaxisISD::ExtractElementF64, DL, MVT::i32, Op.getOperand(0),
                Const1);
  SDValue Y = (TyY == MVT::f32) ?
    DAG.getNode(ISD::BITCAST, DL, MVT::i32, Op.getOperand(1)) :
    DAG.getNode(MaxisISD::ExtractElementF64, DL, MVT::i32, Op.getOperand(1),
                Const1);

  if (HasExtractInsert) {
    // ext  E, Y, 31, 1  ; extract bit31 of Y
    // ins  X, E, 31, 1  ; insert extracted bit at bit31 of X
    SDValue E = DAG.getNode(MaxisISD::Ext, DL, MVT::i32, Y, Const31, Const1);
    Res = DAG.getNode(MaxisISD::Ins, DL, MVT::i32, E, Const31, Const1, X);
  } else {
    // slli SllX, X, 1
    // srl SrlX, SllX, 1
    // srl SrlY, Y, 31
    // slli SllY, SrlX, 31
    // or  Or, SrlX, SlliY
    SDValue SlliX = DAG.getNode(ISD::SHL, DL, MVT::i32, X, Const1);
    SDValue SrlX = DAG.getNode(ISD::SRL, DL, MVT::i32, SlliX, Const1);
    SDValue SrlY = DAG.getNode(ISD::SRL, DL, MVT::i32, Y, Const31);
    SDValue SlliY = DAG.getNode(ISD::SHL, DL, MVT::i32, SrlY, Const31);
    Res = DAG.getNode(ISD::OR, DL, MVT::i32, SrlX, SlliY);
  }

  if (TyX == MVT::f32)
    return DAG.getNode(ISD::BITCAST, DL, Op.getOperand(0).getValueType(), Res);

  SDValue LowX = DAG.getNode(MaxisISD::ExtractElementF64, DL, MVT::i32,
                             Op.getOperand(0),
                             DAG.getConstant(0, DL, MVT::i32));
  return DAG.getNode(MaxisISD::BuildPairF64, DL, MVT::f64, LowX, Res);
}

static SDValue lowerFCOPYSIGN64(SDValue Op, SelectionDAG &DAG,
                                bool HasExtractInsert) {
  unsigned WidthX = Op.getOperand(0).getValueSizeInBits();
  unsigned WidthY = Op.getOperand(1).getValueSizeInBits();
  EVT TyX = MVT::getIntegerVT(WidthX), TyY = MVT::getIntegerVT(WidthY);
  SDLoc DL(Op);
  SDValue Const1 = DAG.getConstant(1, DL, MVT::i32);

  // Bitcast to integer nodes.
  SDValue X = DAG.getNode(ISD::BITCAST, DL, TyX, Op.getOperand(0));
  SDValue Y = DAG.getNode(ISD::BITCAST, DL, TyY, Op.getOperand(1));

  if (HasExtractInsert) {
    // ext  E, Y, width(Y) - 1, 1  ; extract bit width(Y)-1 of Y
    // ins  X, E, width(X) - 1, 1  ; insert extracted bit at bit width(X)-1 of X
    SDValue E = DAG.getNode(MaxisISD::Ext, DL, TyY, Y,
                            DAG.getConstant(WidthY - 1, DL, MVT::i32), Const1);

    if (WidthX > WidthY)
      E = DAG.getNode(ISD::ZERO_EXTEND, DL, TyX, E);
    else if (WidthY > WidthX)
      E = DAG.getNode(ISD::TRUNCATE, DL, TyX, E);

    SDValue I = DAG.getNode(MaxisISD::Ins, DL, TyX, E,
                            DAG.getConstant(WidthX - 1, DL, MVT::i32), Const1,
                            X);
    return DAG.getNode(ISD::BITCAST, DL, Op.getOperand(0).getValueType(), I);
  }

  // (d)slli SlliX, X, 1
  // (d)srl SrlX, SlliX, 1
  // (d)srl SrlY, Y, width(Y)-1
  // (d)slli SlliY, SrlX, width(Y)-1
  // or     Or, SrlX, SlliY
  SDValue SlliX = DAG.getNode(ISD::SHL, DL, TyX, X, Const1);
  SDValue SrlX = DAG.getNode(ISD::SRL, DL, TyX, SlliX, Const1);
  SDValue SrlY = DAG.getNode(ISD::SRL, DL, TyY, Y,
                             DAG.getConstant(WidthY - 1, DL, MVT::i32));

  if (WidthX > WidthY)
    SrlY = DAG.getNode(ISD::ZERO_EXTEND, DL, TyX, SrlY);
  else if (WidthY > WidthX)
    SrlY = DAG.getNode(ISD::TRUNCATE, DL, TyX, SrlY);

  SDValue SlliY = DAG.getNode(ISD::SHL, DL, TyX, SrlY,
                             DAG.getConstant(WidthX - 1, DL, MVT::i32));
  SDValue Or = DAG.getNode(ISD::OR, DL, TyX, SrlX, SlliY);
  return DAG.getNode(ISD::BITCAST, DL, Op.getOperand(0).getValueType(), Or);
}

SDValue
MaxisTargetLowering::lowerFCOPYSIGN(SDValue Op, SelectionDAG &DAG) const {
  if (Subtarget.isGP64bit())
    return lowerFCOPYSIGN64(Op, DAG, Subtarget.hasExtractInsert());

  return lowerFCOPYSIGN32(Op, DAG, Subtarget.hasExtractInsert());
}

SDValue MaxisTargetLowering::
lowerFRAMEADDR(SDValue Op, SelectionDAG &DAG) const {
  // check the depth
  assert((cast<ConstantSDNode>(Op.getOperand(0))->getZExtValue() == 0) &&
         "Frame address can only be determined for current frame.");

  MachineFrameInfo &MFI = DAG.getMachineFunction().getFrameInfo();
  MFI.setFrameAddressIsTaken(true);
  EVT VT = Op.getValueType();
  SDLoc DL(Op);
  SDValue FrameAddr = DAG.getCopyFromReg(
      DAG.getEntryNode(), DL, ABI.IsN64() ? Maxis::FP_64 : Maxis::FP, VT);
  return FrameAddr;
}

SDValue MaxisTargetLowering::lowerRETURNADDR(SDValue Op,
                                            SelectionDAG &DAG) const {
  if (verifyReturnAddressArgumentIsConstant(Op, DAG))
    return SDValue();

  // check the depth
  assert((cast<ConstantSDNode>(Op.getOperand(0))->getZExtValue() == 0) &&
         "Return address can be determined only for current frame.");

  MachineFunction &MF = DAG.getMachineFunction();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  MVT VT = Op.getSimpleValueType();
  unsigned RA = ABI.IsN64() ? Maxis::RA_64 : Maxis::RA;
  MFI.setReturnAddressIsTaken(true);

  // Return RA, which contains the return address. Mark it an implicit live-in.
  unsigned Reg = MF.addLiveIn(RA, getRegClassFor(VT));
  return DAG.getCopyFromReg(DAG.getEntryNode(), SDLoc(Op), Reg, VT);
}

// An EH_RETURN is the result of lowering llvm.eh.return which in turn is
// generated from __builtin_eh_return (offset, handler)
// The effect of this is to adjust the stack pointer by "offset"
// and then branch to "handler".
SDValue MaxisTargetLowering::lowerEH_RETURN(SDValue Op, SelectionDAG &DAG)
                                                                     const {
  MachineFunction &MF = DAG.getMachineFunction();
  MaxisFunctionInfo *MaxisFI = MF.getInfo<MaxisFunctionInfo>();

  MaxisFI->setCallsEhReturn();
  SDValue Chain     = Op.getOperand(0);
  SDValue Offset    = Op.getOperand(1);
  SDValue Handler   = Op.getOperand(2);
  SDLoc DL(Op);
  EVT Ty = ABI.IsN64() ? MVT::i64 : MVT::i32;

  // Store stack offset in V1, store jump target in V0. Glue CopyToReg and
  // EH_RETURN nodes, so that instructions are emitted back-to-back.
  unsigned OffsetReg = ABI.IsN64() ? Maxis::V1_64 : Maxis::V1;
  unsigned AddrReg = ABI.IsN64() ? Maxis::V0_64 : Maxis::V0;
  Chain = DAG.getCopyToReg(Chain, DL, OffsetReg, Offset, SDValue());
  Chain = DAG.getCopyToReg(Chain, DL, AddrReg, Handler, Chain.getValue(1));
  return DAG.getNode(MaxisISD::EH_RETURN, DL, MVT::Other, Chain,
                     DAG.getRegister(OffsetReg, Ty),
                     DAG.getRegister(AddrReg, getPointerTy(MF.getDataLayout())),
                     Chain.getValue(1));
}

SDValue MaxisTargetLowering::lowerATOMIC_FENCE(SDValue Op,
                                              SelectionDAG &DAG) const {
  // FIXME: Need pseudo-fence for 'singlethread' fences
  // FIXME: Set SType for weaker fences where supported/appropriate.
  unsigned SType = 0;
  SDLoc DL(Op);
  return DAG.getNode(MaxisISD::Sync, DL, MVT::Other, Op.getOperand(0),
                     DAG.getConstant(SType, DL, MVT::i32));
}

SDValue MaxisTargetLowering::lowerShiftLeftParts(SDValue Op,
                                                SelectionDAG &DAG) const {
  SDLoc DL(Op);
  MVT VT = Subtarget.isGP64bit() ? MVT::i64 : MVT::i32;

  SDValue Lo = Op.getOperand(0), Hi = Op.getOperand(1);
  SDValue Shamt = Op.getOperand(2);
  // if shamt < (VT.bits):
  //  lo = (shl lo, shamt)
  //  hi = (or (shl hi, shamt) (srl (srl lo, 1), ~shamt))
  // else:
  //  lo = 0
  //  hi = (shl lo, shamt[4:0])
  SDValue Not = DAG.getNode(ISD::XOR, DL, MVT::i32, Shamt,
                            DAG.getConstant(-1, DL, MVT::i32));
  SDValue ShiftRight1Lo = DAG.getNode(ISD::SRL, DL, VT, Lo,
                                      DAG.getConstant(1, DL, VT));
  SDValue ShiftRightLo = DAG.getNode(ISD::SRL, DL, VT, ShiftRight1Lo, Not);
  SDValue ShiftLeftHi = DAG.getNode(ISD::SHL, DL, VT, Hi, Shamt);
  SDValue Or = DAG.getNode(ISD::OR, DL, VT, ShiftLeftHi, ShiftRightLo);
  SDValue ShiftLeftLo = DAG.getNode(ISD::SHL, DL, VT, Lo, Shamt);
  SDValue Cond = DAG.getNode(ISD::AND, DL, MVT::i32, Shamt,
                             DAG.getConstant(VT.getSizeInBits(), DL, MVT::i32));
  Lo = DAG.getNode(ISD::SELECT, DL, VT, Cond,
                   DAG.getConstant(0, DL, VT), ShiftLeftLo);
  Hi = DAG.getNode(ISD::SELECT, DL, VT, Cond, ShiftLeftLo, Or);

  SDValue Ops[2] = {Lo, Hi};
  return DAG.getMergeValues(Ops, DL);
}

SDValue MaxisTargetLowering::lowerShiftRightParts(SDValue Op, SelectionDAG &DAG,
                                                 bool IsSRA) const {
  SDLoc DL(Op);
  SDValue Lo = Op.getOperand(0), Hi = Op.getOperand(1);
  SDValue Shamt = Op.getOperand(2);
  MVT VT = Subtarget.isGP64bit() ? MVT::i64 : MVT::i32;

  // if shamt < (VT.bits):
  //  lo = (or (shl (shl hi, 1), ~shamt) (srl lo, shamt))
  //  if isSRA:
  //    hi = (sra hi, shamt)
  //  else:
  //    hi = (srl hi, shamt)
  // else:
  //  if isSRA:
  //   lo = (sra hi, shamt[4:0])
  //   hi = (sra hi, 31)
  //  else:
  //   lo = (srl hi, shamt[4:0])
  //   hi = 0
  SDValue Not = DAG.getNode(ISD::XOR, DL, MVT::i32, Shamt,
                            DAG.getConstant(-1, DL, MVT::i32));
  SDValue ShiftLeft1Hi = DAG.getNode(ISD::SHL, DL, VT, Hi,
                                     DAG.getConstant(1, DL, VT));
  SDValue ShiftLeftHi = DAG.getNode(ISD::SHL, DL, VT, ShiftLeft1Hi, Not);
  SDValue ShiftRightLo = DAG.getNode(ISD::SRL, DL, VT, Lo, Shamt);
  SDValue Or = DAG.getNode(ISD::OR, DL, VT, ShiftLeftHi, ShiftRightLo);
  SDValue ShiftRightHi = DAG.getNode(IsSRA ? ISD::SRA : ISD::SRL,
                                     DL, VT, Hi, Shamt);
  SDValue Cond = DAG.getNode(ISD::AND, DL, MVT::i32, Shamt,
                             DAG.getConstant(VT.getSizeInBits(), DL, MVT::i32));
  SDValue Ext = DAG.getNode(ISD::SRA, DL, VT, Hi,
                            DAG.getConstant(VT.getSizeInBits() - 1, DL, VT));
  Lo = DAG.getNode(ISD::SELECT, DL, VT, Cond, ShiftRightHi, Or);
  Hi = DAG.getNode(ISD::SELECT, DL, VT, Cond,
                   IsSRA ? Ext : DAG.getConstant(0, DL, VT), ShiftRightHi);

  SDValue Ops[2] = {Lo, Hi};
  return DAG.getMergeValues(Ops, DL);
}

static SDValue createLoadLR(unsigned Opc, SelectionDAG &DAG, LoadSDNode *LD,
                            SDValue Chain, SDValue Src, unsigned Offset) {
  SDValue Ptr = LD->getBasePtr();
  EVT VT = LD->getValueType(0), MemVT = LD->getMemoryVT();
  EVT BasePtrVT = Ptr.getValueType();
  SDLoc DL(LD);
  SDVTList VTList = DAG.getVTList(VT, MVT::Other);

  if (Offset)
    Ptr = DAG.getNode(ISD::ADD, DL, BasePtrVT, Ptr,
                      DAG.getConstant(Offset, DL, BasePtrVT));

  SDValue Ops[] = { Chain, Ptr, Src };
  return DAG.getMemIntrinsicNode(Opc, DL, VTList, Ops, MemVT,
                                 LD->getMemOperand());
}

// Expand an unaligned 32 or 64-bit integer load node.
SDValue MaxisTargetLowering::lowerLOAD(SDValue Op, SelectionDAG &DAG) const {
  LoadSDNode *LD = cast<LoadSDNode>(Op);
  EVT MemVT = LD->getMemoryVT();

  if (Subtarget.systemSupportsUnalignedAccess())
    return Op;

  // Return if load is aligned or if MemVT is neither i32 nor i64.
  if ((LD->getAlignment() >= MemVT.getSizeInBits() / 8) ||
      ((MemVT != MVT::i32) && (MemVT != MVT::i64)))
    return SDValue();

  bool IsLittle = Subtarget.isLittle();
  EVT VT = Op.getValueType();
  ISD::LoadExtType ExtType = LD->getExtensionType();
  SDValue Chain = LD->getChain(), Undef = DAG.getUNDEF(VT);

  assert((VT == MVT::i32) || (VT == MVT::i64));

  // Expand
  //  (set dst, (i64 (load baseptr)))
  // to
  //  (set tmp, (ldl (add baseptr, 7), undef))
  //  (set dst, (ldr baseptr, tmp))
  if ((VT == MVT::i64) && (ExtType == ISD::NON_EXTLOAD)) {
    SDValue LDL = createLoadLR(MaxisISD::LDL, DAG, LD, Chain, Undef,
                               IsLittle ? 7 : 0);
    return createLoadLR(MaxisISD::LDR, DAG, LD, LDL.getValue(1), LDL,
                        IsLittle ? 0 : 7);
  }

  SDValue LWL = createLoadLR(MaxisISD::LWL, DAG, LD, Chain, Undef,
                             IsLittle ? 3 : 0);
  SDValue LWR = createLoadLR(MaxisISD::LWR, DAG, LD, LWL.getValue(1), LWL,
                             IsLittle ? 0 : 3);

  // Expand
  //  (set dst, (i32 (load baseptr))) or
  //  (set dst, (i64 (sextload baseptr))) or
  //  (set dst, (i64 (extload baseptr)))
  // to
  //  (set tmp, (lwl (add baseptr, 3), undef))
  //  (set dst, (lwr baseptr, tmp))
  if ((VT == MVT::i32) || (ExtType == ISD::SEXTLOAD) ||
      (ExtType == ISD::EXTLOAD))
    return LWR;

  assert((VT == MVT::i64) && (ExtType == ISD::ZEXTLOAD));

  // Expand
  //  (set dst, (i64 (zextload baseptr)))
  // to
  //  (set tmp0, (lwl (add baseptr, 3), undef))
  //  (set tmp1, (lwr baseptr, tmp0))
  //  (set tmp2, (shl tmp1, 32))
  //  (set dst, (srl tmp2, 32))
  SDLoc DL(LD);
  SDValue Const32 = DAG.getConstant(32, DL, MVT::i32);
  SDValue SLLi = DAG.getNode(ISD::SHL, DL, MVT::i64, LWR, Const32);
  SDValue SRL = DAG.getNode(ISD::SRL, DL, MVT::i64, SLLi, Const32);
  SDValue Ops[] = { SRL, LWR.getValue(1) };
  return DAG.getMergeValues(Ops, DL);
}

static SDValue createStoreLR(unsigned Opc, SelectionDAG &DAG, StoreSDNode *SD,
                             SDValue Chain, unsigned Offset) {
  SDValue Ptr = SD->getBasePtr(), Value = SD->getValue();
  EVT MemVT = SD->getMemoryVT(), BasePtrVT = Ptr.getValueType();
  SDLoc DL(SD);
  SDVTList VTList = DAG.getVTList(MVT::Other);

  if (Offset)
    Ptr = DAG.getNode(ISD::ADD, DL, BasePtrVT, Ptr,
                      DAG.getConstant(Offset, DL, BasePtrVT));

  SDValue Ops[] = { Chain, Value, Ptr };
  return DAG.getMemIntrinsicNode(Opc, DL, VTList, Ops, MemVT,
                                 SD->getMemOperand());
}

// Expand an unaligned 32 or 64-bit integer store node.
static SDValue lowerUnalignedIntStore(StoreSDNode *SD, SelectionDAG &DAG,
                                      bool IsLittle) {
  SDValue Value = SD->getValue(), Chain = SD->getChain();
  EVT VT = Value.getValueType();

  // Expand
  //  (store val, baseptr) or
  //  (truncstore val, baseptr)
  // to
  //  (swl val, (add baseptr, 3))
  //  (swr val, baseptr)
  if ((VT == MVT::i32) || SD->isTruncatingStore()) {
    SDValue SWL = createStoreLR(MaxisISD::SWL, DAG, SD, Chain,
                                IsLittle ? 3 : 0);
    return createStoreLR(MaxisISD::SWR, DAG, SD, SWL, IsLittle ? 0 : 3);
  }

  assert(VT == MVT::i64);

  // Expand
  //  (store val, baseptr)
  // to
  //  (sdl val, (add baseptr, 7))
  //  (sdr val, baseptr)
  SDValue SDL = createStoreLR(MaxisISD::SDL, DAG, SD, Chain, IsLittle ? 7 : 0);
  return createStoreLR(MaxisISD::SDR, DAG, SD, SDL, IsLittle ? 0 : 7);
}

// Lower (store (fp_to_sint $fp) $ptr) to (store (TruncIntFP $fp), $ptr).
static SDValue lowerFP_TO_SINT_STORE(StoreSDNode *SD, SelectionDAG &DAG) {
  SDValue Val = SD->getValue();

  if (Val.getOpcode() != ISD::FP_TO_SINT)
    return SDValue();

  EVT FPTy = EVT::getFloatingPointVT(Val.getValueSizeInBits());
  SDValue Tr = DAG.getNode(MaxisISD::TruncIntFP, SDLoc(Val), FPTy,
                           Val.getOperand(0));
  return DAG.getStore(SD->getChain(), SDLoc(SD), Tr, SD->getBasePtr(),
                      SD->getPointerInfo(), SD->getAlignment(),
                      SD->getMemOperand()->getFlags());
}

SDValue MaxisTargetLowering::lowerSTORE(SDValue Op, SelectionDAG &DAG) const {
  StoreSDNode *SD = cast<StoreSDNode>(Op);
  EVT MemVT = SD->getMemoryVT();

  // Lower unaligned integer stores.
  if (!Subtarget.systemSupportsUnalignedAccess() &&
      (SD->getAlignment() < MemVT.getSizeInBits() / 8) &&
      ((MemVT == MVT::i32) || (MemVT == MVT::i64)))
    return lowerUnalignedIntStore(SD, DAG, Subtarget.isLittle());

  return lowerFP_TO_SINT_STORE(SD, DAG);
}

SDValue MaxisTargetLowering::lowerEH_DWARF_CFA(SDValue Op,
                                              SelectionDAG &DAG) const {

  // Return a fixed StackObject with offset 0 which points to the old stack
  // pointer.
  MachineFrameInfo &MFI = DAG.getMachineFunction().getFrameInfo();
  EVT ValTy = Op->getValueType(0);
  int FI = MFI.CreateFixedObject(Op.getValueSizeInBits() / 8, 0, false);
  return DAG.getFrameIndex(FI, ValTy);
}

SDValue MaxisTargetLowering::lowerFP_TO_SINT(SDValue Op,
                                            SelectionDAG &DAG) const {
  EVT FPTy = EVT::getFloatingPointVT(Op.getValueSizeInBits());
  SDValue Trunc = DAG.getNode(MaxisISD::TruncIntFP, SDLoc(Op), FPTy,
                              Op.getOperand(0));
  return DAG.getNode(ISD::BITCAST, SDLoc(Op), Op.getValueType(), Trunc);
}

//===----------------------------------------------------------------------===//
//                      Calling Convention Implementation
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// TODO: Implement a generic logic using tblgen that can support this.
// Maxis O32 ABI rules:
// ---
// i32 - Passed in A0, A1, A2, A3 and stack
// f32 - Only passed in f32 registers if no int reg has been used yet to hold
//       an argument. Otherwise, passed in A1, A2, A3 and stack.
// f64 - Only passed in two aliased f32 registers if no int reg has been used
//       yet to hold an argument. Otherwise, use A2, A3 and stack. If A1 is
//       not used, it must be shadowed. If only A3 is available, shadow it and
//       go to stack.
// vXiX - Received as scalarized i32s, passed in A0 - A3 and the stack.
// vXf32 - Passed in either a pair of registers {A0, A1}, {A2, A3} or {A0 - A3}
//         with the remainder spilled to the stack.
// vXf64 - Passed in either {A0, A1, A2, A3} or {A2, A3} and in both cases
//         spilling the remainder to the stack.
//
//  For vararg functions, all arguments are passed in A0, A1, A2, A3 and stack.
//===----------------------------------------------------------------------===//

static bool CC_MaxisO32(unsigned ValNo, MVT ValVT, MVT LocVT,
                       CCValAssign::LocInfo LocInfo, ISD::ArgFlagsTy ArgFlags,
                       CCState &State, ArrayRef<MCPhysReg> F64Regs) {
  const MaxisSubtarget &Subtarget = static_cast<const MaxisSubtarget &>(
      State.getMachineFunction().getSubtarget());

  static const MCPhysReg IntRegs[] = { Maxis::A0, Maxis::A1, Maxis::A2, Maxis::A3 };

  const MaxisCCState * MaxisState = static_cast<MaxisCCState *>(&State);

  static const MCPhysReg F32Regs[] = { Maxis::F12, Maxis::F14 };

  static const MCPhysReg FloatVectorIntRegs[] = { Maxis::A0, Maxis::A2 };

  // Do not process byval args here.
  if (ArgFlags.isByVal())
    return true;

  // Promote i8 and i16
  if (ArgFlags.isInReg() && !Subtarget.isLittle()) {
    if (LocVT == MVT::i8 || LocVT == MVT::i16 || LocVT == MVT::i32) {
      LocVT = MVT::i32;
      if (ArgFlags.isSExt())
        LocInfo = CCValAssign::SExtUpper;
      else if (ArgFlags.isZExt())
        LocInfo = CCValAssign::ZExtUpper;
      else
        LocInfo = CCValAssign::AExtUpper;
    }
  }

  // Promote i8 and i16
  if (LocVT == MVT::i8 || LocVT == MVT::i16) {
    LocVT = MVT::i32;
    if (ArgFlags.isSExt())
      LocInfo = CCValAssign::SExt;
    else if (ArgFlags.isZExt())
      LocInfo = CCValAssign::ZExt;
    else
      LocInfo = CCValAssign::AExt;
  }

  unsigned Reg;

  // f32 and f64 are allocated in A0, A1, A2, A3 when either of the following
  // is true: function is vararg, argument is 3rd or higher, there is previous
  // argument which is not f32 or f64.
  bool AllocateFloatsInIntReg = State.isVarArg() || ValNo > 1 ||
                                State.getFirstUnallocated(F32Regs) != ValNo;
  unsigned OrigAlign = ArgFlags.getOrigAlign();
  bool isI64 = (ValVT == MVT::i32 && OrigAlign == 8);
  bool isVectorFloat = MaxisState->WasOriginalArgVectorFloat(ValNo);

  // The MAXIS vector ABI for floats passes them in a pair of registers
  if (ValVT == MVT::i32 && isVectorFloat) {
    // This is the start of an vector that was scalarized into an unknown number
    // of components. It doesn't matter how many there are. Allocate one of the
    // notional 8 byte aligned registers which map onto the argument stack, and
    // shadow the register lost to alignment requirements.
    if (ArgFlags.isSplit()) {
      Reg = State.AllocateReg(FloatVectorIntRegs);
      if (Reg == Maxis::A2)
        State.AllocateReg(Maxis::A1);
      else if (Reg == 0)
        State.AllocateReg(Maxis::A3);
    } else {
      // If we're an intermediate component of the split, we can just attempt to
      // allocate a register directly.
      Reg = State.AllocateReg(IntRegs);
    }
  } else if (ValVT == MVT::i32 || (ValVT == MVT::f32 && AllocateFloatsInIntReg)) {
    Reg = State.AllocateReg(IntRegs);
    // If this is the first part of an i64 arg,
    // the allocated register must be either A0 or A2.
    if (isI64 && (Reg == Maxis::A1 || Reg == Maxis::A3))
      Reg = State.AllocateReg(IntRegs);
    LocVT = MVT::i32;
  } else if (ValVT == MVT::f64 && AllocateFloatsInIntReg) {
    // Allocate int register and shadow next int register. If first
    // available register is Maxis::A1 or Maxis::A3, shadow it too.
    Reg = State.AllocateReg(IntRegs);
    if (Reg == Maxis::A1 || Reg == Maxis::A3)
      Reg = State.AllocateReg(IntRegs);
    State.AllocateReg(IntRegs);
    LocVT = MVT::i32;
  } else if (ValVT.isFloatingPoint() && !AllocateFloatsInIntReg) {
    // we are guaranteed to find an available float register
    if (ValVT == MVT::f32) {
      Reg = State.AllocateReg(F32Regs);
      // Shadow int register
      State.AllocateReg(IntRegs);
    } else {
      Reg = State.AllocateReg(F64Regs);
      // Shadow int registers
      unsigned Reg2 = State.AllocateReg(IntRegs);
      if (Reg2 == Maxis::A1 || Reg2 == Maxis::A3)
        State.AllocateReg(IntRegs);
      State.AllocateReg(IntRegs);
    }
  } else
    llvm_unreachable("Cannot handle this ValVT.");

  if (!Reg) {
    unsigned Offset = State.AllocateStack(ValVT.getStoreSize(), OrigAlign);
    State.addLoc(CCValAssign::getMem(ValNo, ValVT, Offset, LocVT, LocInfo));
  } else
    State.addLoc(CCValAssign::getReg(ValNo, ValVT, Reg, LocVT, LocInfo));

  return false;
}

static bool CC_MaxisO32_FP32(unsigned ValNo, MVT ValVT,
                            MVT LocVT, CCValAssign::LocInfo LocInfo,
                            ISD::ArgFlagsTy ArgFlags, CCState &State) {
  static const MCPhysReg F64Regs[] = { Maxis::D6, Maxis::D7 };

  return CC_MaxisO32(ValNo, ValVT, LocVT, LocInfo, ArgFlags, State, F64Regs);
}

static bool CC_MaxisO32_FP64(unsigned ValNo, MVT ValVT,
                            MVT LocVT, CCValAssign::LocInfo LocInfo,
                            ISD::ArgFlagsTy ArgFlags, CCState &State) {
  static const MCPhysReg F64Regs[] = { Maxis::D12_64, Maxis::D14_64 };

  return CC_MaxisO32(ValNo, ValVT, LocVT, LocInfo, ArgFlags, State, F64Regs);
}

static bool CC_MaxisO32(unsigned ValNo, MVT ValVT, MVT LocVT,
                       CCValAssign::LocInfo LocInfo, ISD::ArgFlagsTy ArgFlags,
                       CCState &State) LLVM_ATTRIBUTE_UNUSED;

#include "MaxisGenCallingConv.inc"

//===----------------------------------------------------------------------===//
//                  Call Calling Convention Implementation
//===----------------------------------------------------------------------===//

// Return next O32 integer argument register.
static unsigned getNextIntArgReg(unsigned Reg) {
  assert((Reg == Maxis::A0) || (Reg == Maxis::A2));
  return (Reg == Maxis::A0) ? Maxis::A1 : Maxis::A3;
}

SDValue MaxisTargetLowering::passArgOnStack(SDValue StackPtr, unsigned Offset,
                                           SDValue Chain, SDValue Arg,
                                           const SDLoc &DL, bool IsTailCall,
                                           SelectionDAG &DAG) const {
  if (!IsTailCall) {
    SDValue PtrOff =
        DAG.getNode(ISD::ADD, DL, getPointerTy(DAG.getDataLayout()), StackPtr,
                    DAG.getIntPtrConstant(Offset, DL));
    return DAG.getStore(Chain, DL, Arg, PtrOff, MachinePointerInfo());
  }

  MachineFrameInfo &MFI = DAG.getMachineFunction().getFrameInfo();
  int FI = MFI.CreateFixedObject(Arg.getValueSizeInBits() / 8, Offset, false);
  SDValue FIN = DAG.getFrameIndex(FI, getPointerTy(DAG.getDataLayout()));
  return DAG.getStore(Chain, DL, Arg, FIN, MachinePointerInfo(),
                      /* Alignment = */ 0, MachineMemOperand::MOVolatile);
}

void MaxisTargetLowering::
getOpndList(SmallVectorImpl<SDValue> &Ops,
            std::deque<std::pair<unsigned, SDValue>> &RegsToPass,
            bool IsPICCall, bool GlobalOrExternal, bool InternalLinkage,
            bool IsCallReloc, CallLoweringInfo &CLI, SDValue Callee,
            SDValue Chain) const {
  // Insert node "GP copy globalreg" before call to function.
  //
  // R_MAXIS_CALL* operators (emitted when non-internal functions are called
  // in PIC mode) allow symbols to be resolved via lazy binding.
  // The lazy binding stub requires GP to point to the GOT.
  // Note that we don't need GP to point to the GOT for indirect calls
  // (when R_MAXIS_CALL* is not used for the call) because Maxis linker generates
  // lazy binding stub for a function only when R_MAXIS_CALL* are the only relocs
  // used for the function (that is, Maxis linker doesn't generate lazy binding
  // stub for a function whose address is taken in the program).
  if (IsPICCall && !InternalLinkage && IsCallReloc) {
    unsigned GPReg = ABI.IsN64() ? Maxis::GP_64 : Maxis::GP;
    EVT Ty = ABI.IsN64() ? MVT::i64 : MVT::i32;
    RegsToPass.push_back(std::make_pair(GPReg, getGlobalReg(CLI.DAG, Ty)));
  }

  // Build a sequence of copy-to-reg nodes chained together with token
  // chain and flag operands which copy the outgoing args into registers.
  // The InFlag in necessary since all emitted instructions must be
  // stuck together.
  SDValue InFlag;

  for (unsigned i = 0, e = RegsToPass.size(); i != e; ++i) {
    Chain = CLI.DAG.getCopyToReg(Chain, CLI.DL, RegsToPass[i].first,
                                 RegsToPass[i].second, InFlag);
    InFlag = Chain.getValue(1);
  }

  // Add argument registers to the end of the list so that they are
  // known live into the call.
  for (unsigned i = 0, e = RegsToPass.size(); i != e; ++i)
    Ops.push_back(CLI.DAG.getRegister(RegsToPass[i].first,
                                      RegsToPass[i].second.getValueType()));

  // Add a register mask operand representing the call-preserved registers.
  const TargetRegisterInfo *TRI = Subtarget.getRegisterInfo();
  const uint32_t *Mask =
      TRI->getCallPreservedMask(CLI.DAG.getMachineFunction(), CLI.CallConv);
  assert(Mask && "Missing call preserved mask for calling convention");
  if (Subtarget.inMaxis16HardFloat()) {
    if (GlobalAddressSDNode *G = dyn_cast<GlobalAddressSDNode>(CLI.Callee)) {
      StringRef Sym = G->getGlobal()->getName();
      Function *F = G->getGlobal()->getParent()->getFunction(Sym);
      if (F && F->hasFnAttribute("__Maxis16RetHelper")) {
        Mask = MaxisRegisterInfo::getMaxis16RetHelperMask();
      }
    }
  }
  Ops.push_back(CLI.DAG.getRegisterMask(Mask));

  if (InFlag.getNode())
    Ops.push_back(InFlag);
}

/// LowerCall - functions arguments are copied from virtual regs to
/// (physical regs)/(stack frame), CALLSEQ_START and CALLSEQ_END are emitted.
SDValue
MaxisTargetLowering::LowerCall(TargetLowering::CallLoweringInfo &CLI,
                              SmallVectorImpl<SDValue> &InVals) const {
  SelectionDAG &DAG                     = CLI.DAG;
  SDLoc DL                              = CLI.DL;
  SmallVectorImpl<ISD::OutputArg> &Outs = CLI.Outs;
  SmallVectorImpl<SDValue> &OutVals     = CLI.OutVals;
  SmallVectorImpl<ISD::InputArg> &Ins   = CLI.Ins;
  SDValue Chain                         = CLI.Chain;
  SDValue Callee                        = CLI.Callee;
  bool &IsTailCall                      = CLI.IsTailCall;
  CallingConv::ID CallConv              = CLI.CallConv;
  bool IsVarArg                         = CLI.IsVarArg;

  MachineFunction &MF = DAG.getMachineFunction();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  const TargetFrameLowering *TFL = Subtarget.getFrameLowering();
  MaxisFunctionInfo *FuncInfo = MF.getInfo<MaxisFunctionInfo>();
  bool IsPIC = isPositionIndependent();

  // Analyze operands of the call, assigning locations to each operand.
  SmallVector<CCValAssign, 16> ArgLocs;
  MaxisCCState CCInfo(
      CallConv, IsVarArg, DAG.getMachineFunction(), ArgLocs, *DAG.getContext(),
      MaxisCCState::getSpecialCallingConvForCallee(Callee.getNode(), Subtarget));

  // Allocate the reserved argument area. It seems strange to do this from the
  // caller side but removing it breaks the frame size calculation.
  CCInfo.AllocateStack(ABI.GetCalleeAllocdArgSizeInBytes(CallConv), 1);

  const ExternalSymbolSDNode *ES =
      dyn_cast_or_null<const ExternalSymbolSDNode>(Callee.getNode());
  CCInfo.AnalyzeCallOperands(Outs, CC_Maxis, CLI.getArgs(),
                             ES ? ES->getSymbol() : nullptr);

  // Get a count of how many bytes are to be pushed on the stack.
  unsigned NextStackOffset = CCInfo.getNextStackOffset();

  // Check if it's really possible to do a tail call. Restrict it to functions
  // that are part of this compilation unit.
  bool InternalLinkage = false;
  if (IsTailCall) {
    IsTailCall = isEligibleForTailCallOptimization(
        CCInfo, NextStackOffset, *MF.getInfo<MaxisFunctionInfo>());
     if (GlobalAddressSDNode *G = dyn_cast<GlobalAddressSDNode>(Callee)) {
      InternalLinkage = G->getGlobal()->hasInternalLinkage();
      IsTailCall &= (InternalLinkage || G->getGlobal()->hasLocalLinkage() ||
                     G->getGlobal()->hasPrivateLinkage() ||
                     G->getGlobal()->hasHiddenVisibility() ||
                     G->getGlobal()->hasProtectedVisibility());
     }
  }
  if (!IsTailCall && CLI.CS && CLI.CS.isMustTailCall())
    report_fatal_error("failed to perform tail call elimination on a call "
                       "site marked musttail");

  if (IsTailCall)
    ++NumTailCalls;

  // Chain is the output chain of the last Load/Store or CopyToReg node.
  // ByValChain is the output chain of the last Memcpy node created for copying
  // byval arguments to the stack.
  unsigned StackAlignment = TFL->getStackAlignment();
  NextStackOffset = alignTo(NextStackOffset, StackAlignment);
  SDValue NextStackOffsetVal = DAG.getIntPtrConstant(NextStackOffset, DL, true);

  if (!IsTailCall)
    Chain = DAG.getCALLSEQ_START(Chain, NextStackOffset, 0, DL);

  SDValue StackPtr =
      DAG.getCopyFromReg(Chain, DL, ABI.IsN64() ? Maxis::SP_64 : Maxis::SP,
                         getPointerTy(DAG.getDataLayout()));

  std::deque<std::pair<unsigned, SDValue>> RegsToPass;
  SmallVector<SDValue, 8> MemOpChains;

  CCInfo.rewindByValRegsInfo();

  // Walk the register/memloc assignments, inserting copies/loads.
  for (unsigned i = 0, e = ArgLocs.size(); i != e; ++i) {
    SDValue Arg = OutVals[i];
    CCValAssign &VA = ArgLocs[i];
    MVT ValVT = VA.getValVT(), LocVT = VA.getLocVT();
    ISD::ArgFlagsTy Flags = Outs[i].Flags;
    bool UseUpperBits = false;

    // ByVal Arg.
    if (Flags.isByVal()) {
      unsigned FirstByValReg, LastByValReg;
      unsigned ByValIdx = CCInfo.getInRegsParamsProcessed();
      CCInfo.getInRegsParamInfo(ByValIdx, FirstByValReg, LastByValReg);

      assert(Flags.getByValSize() &&
             "ByVal args of size 0 should have been ignored by front-end.");
      assert(ByValIdx < CCInfo.getInRegsParamsCount());
      assert(!IsTailCall &&
             "Do not tail-call optimize if there is a byval argument.");
      passByValArg(Chain, DL, RegsToPass, MemOpChains, StackPtr, MFI, DAG, Arg,
                   FirstByValReg, LastByValReg, Flags, Subtarget.isLittle(),
                   VA);
      CCInfo.nextInRegsParam();
      continue;
    }

    // Promote the value if needed.
    switch (VA.getLocInfo()) {
    default:
      llvm_unreachable("Unknown loc info!");
    case CCValAssign::Full:
      if (VA.isRegLoc()) {
        if ((ValVT == MVT::f32 && LocVT == MVT::i32) ||
            (ValVT == MVT::f64 && LocVT == MVT::i64) ||
            (ValVT == MVT::i64 && LocVT == MVT::f64))
          Arg = DAG.getNode(ISD::BITCAST, DL, LocVT, Arg);
        else if (ValVT == MVT::f64 && LocVT == MVT::i32) {
          SDValue Lo = DAG.getNode(MaxisISD::ExtractElementF64, DL, MVT::i32,
                                   Arg, DAG.getConstant(0, DL, MVT::i32));
          SDValue Hi = DAG.getNode(MaxisISD::ExtractElementF64, DL, MVT::i32,
                                   Arg, DAG.getConstant(1, DL, MVT::i32));
          if (!Subtarget.isLittle())
            std::swap(Lo, Hi);
          unsigned LocRegLo = VA.getLocReg();
          unsigned LocRegHigh = getNextIntArgReg(LocRegLo);
          RegsToPass.push_back(std::make_pair(LocRegLo, Lo));
          RegsToPass.push_back(std::make_pair(LocRegHigh, Hi));
          continue;
        }
      }
      break;
    case CCValAssign::BCvt:
      Arg = DAG.getNode(ISD::BITCAST, DL, LocVT, Arg);
      break;
    case CCValAssign::SExtUpper:
      UseUpperBits = true;
      LLVM_FALLTHROUGH;
    case CCValAssign::SExt:
      Arg = DAG.getNode(ISD::SIGN_EXTEND, DL, LocVT, Arg);
      break;
    case CCValAssign::ZExtUpper:
      UseUpperBits = true;
      LLVM_FALLTHROUGH;
    case CCValAssign::ZExt:
      Arg = DAG.getNode(ISD::ZERO_EXTEND, DL, LocVT, Arg);
      break;
    case CCValAssign::AExtUpper:
      UseUpperBits = true;
      LLVM_FALLTHROUGH;
    case CCValAssign::AExt:
      Arg = DAG.getNode(ISD::ANY_EXTEND, DL, LocVT, Arg);
      break;
    }

    if (UseUpperBits) {
      unsigned ValSizeInBits = Outs[i].ArgVT.getSizeInBits();
      unsigned LocSizeInBits = VA.getLocVT().getSizeInBits();
      Arg = DAG.getNode(
          ISD::SHL, DL, VA.getLocVT(), Arg,
          DAG.getConstant(LocSizeInBits - ValSizeInBits, DL, VA.getLocVT()));
    }

    // Arguments that can be passed on register must be kept at
    // RegsToPass vector
    if (VA.isRegLoc()) {
      RegsToPass.push_back(std::make_pair(VA.getLocReg(), Arg));
      continue;
    }

    // Register can't get to this point...
    assert(VA.isMemLoc());

    // emit ISD::STORE whichs stores the
    // parameter value to a stack Location
    MemOpChains.push_back(passArgOnStack(StackPtr, VA.getLocMemOffset(),
                                         Chain, Arg, DL, IsTailCall, DAG));
  }

  // Transform all store nodes into one single node because all store
  // nodes are independent of each other.
  if (!MemOpChains.empty())
    Chain = DAG.getNode(ISD::TokenFactor, DL, MVT::Other, MemOpChains);

  // If the callee is a GlobalAddress/ExternalSymbol node (quite common, every
  // direct call is) turn it into a TargetGlobalAddress/TargetExternalSymbol
  // node so that legalize doesn't hack it.

  EVT Ty = Callee.getValueType();
  bool GlobalOrExternal = false, IsCallReloc = false;

  // The long-calls feature is ignored in case of PIC.
  // While we do not support -mshared / -mno-shared properly,
  // ignore long-calls in case of -mabicalls too.
  if (!Subtarget.isABICalls() && !IsPIC) {
    // If the function should be called using "long call",
    // get its address into a register to prevent using
    // of the `jal` instruction for the direct call.
    if (auto *N = dyn_cast<ExternalSymbolSDNode>(Callee)) {
      if (Subtarget.useLongCalls())
        Callee = Subtarget.hasSym32()
                     ? getAddrNonPIC(N, SDLoc(N), Ty, DAG)
                     : getAddrNonPICSym64(N, SDLoc(N), Ty, DAG);
    } else if (auto *N = dyn_cast<GlobalAddressSDNode>(Callee)) {
      bool UseLongCalls = Subtarget.useLongCalls();
      // If the function has long-call/far/near attribute
      // it overrides command line switch pased to the backend.
      if (auto *F = dyn_cast<Function>(N->getGlobal())) {
        if (F->hasFnAttribute("long-call"))
          UseLongCalls = true;
        else if (F->hasFnAttribute("short-call"))
          UseLongCalls = false;
      }
      if (UseLongCalls)
        Callee = Subtarget.hasSym32()
                     ? getAddrNonPIC(N, SDLoc(N), Ty, DAG)
                     : getAddrNonPICSym64(N, SDLoc(N), Ty, DAG);
    }
  }

  if (GlobalAddressSDNode *G = dyn_cast<GlobalAddressSDNode>(Callee)) {
    if (IsPIC) {
      const GlobalValue *Val = G->getGlobal();
      InternalLinkage = Val->hasInternalLinkage();

      if (InternalLinkage)
        Callee = getAddrLocal(G, DL, Ty, DAG, ABI.IsN32() || ABI.IsN64());
      else if (LargeGOT) {
        Callee = getAddrGlobalLargeGOT(G, DL, Ty, DAG, MaxisII::MO_CALL_HI16,
                                       MaxisII::MO_CALL_LO16, Chain,
                                       FuncInfo->callPtrInfo(Val));
        IsCallReloc = true;
      } else {
        Callee = getAddrGlobal(G, DL, Ty, DAG, MaxisII::MO_GOT_CALL, Chain,
                               FuncInfo->callPtrInfo(Val));
        IsCallReloc = true;
      }
    } else
      Callee = DAG.getTargetGlobalAddress(G->getGlobal(), DL,
                                          getPointerTy(DAG.getDataLayout()), 0,
                                          MaxisII::MO_NO_FLAG);
    GlobalOrExternal = true;
  }
  else if (ExternalSymbolSDNode *S = dyn_cast<ExternalSymbolSDNode>(Callee)) {
    const char *Sym = S->getSymbol();

    if (!IsPIC) // static
      Callee = DAG.getTargetExternalSymbol(
          Sym, getPointerTy(DAG.getDataLayout()), MaxisII::MO_NO_FLAG);
    else if (LargeGOT) {
      Callee = getAddrGlobalLargeGOT(S, DL, Ty, DAG, MaxisII::MO_CALL_HI16,
                                     MaxisII::MO_CALL_LO16, Chain,
                                     FuncInfo->callPtrInfo(Sym));
      IsCallReloc = true;
    } else { // PIC
      Callee = getAddrGlobal(S, DL, Ty, DAG, MaxisII::MO_GOT_CALL, Chain,
                             FuncInfo->callPtrInfo(Sym));
      IsCallReloc = true;
    }

    GlobalOrExternal = true;
  }

  SmallVector<SDValue, 8> Ops(1, Chain);
  SDVTList NodeTys = DAG.getVTList(MVT::Other, MVT::Glue);

  getOpndList(Ops, RegsToPass, IsPIC, GlobalOrExternal, InternalLinkage,
              IsCallReloc, CLI, Callee, Chain);

  if (IsTailCall) {
    MF.getFrameInfo().setHasTailCall();
    return DAG.getNode(MaxisISD::TailCall, DL, MVT::Other, Ops);
  }

  Chain = DAG.getNode(MaxisISD::JmpLink, DL, NodeTys, Ops);
  SDValue InFlag = Chain.getValue(1);

  // Create the CALLSEQ_END node.
  Chain = DAG.getCALLSEQ_END(Chain, NextStackOffsetVal,
                             DAG.getIntPtrConstant(0, DL, true), InFlag, DL);
  InFlag = Chain.getValue(1);

  // Handle result values, copying them out of physregs into vregs that we
  // return.
  return LowerCallResult(Chain, InFlag, CallConv, IsVarArg, Ins, DL, DAG,
                         InVals, CLI);
}

/// LowerCallResult - Lower the result values of a call into the
/// appropriate copies out of appropriate physical registers.
SDValue MaxisTargetLowering::LowerCallResult(
    SDValue Chain, SDValue InFlag, CallingConv::ID CallConv, bool IsVarArg,
    const SmallVectorImpl<ISD::InputArg> &Ins, const SDLoc &DL,
    SelectionDAG &DAG, SmallVectorImpl<SDValue> &InVals,
    TargetLowering::CallLoweringInfo &CLI) const {
  // Assign locations to each value returned by this call.
  SmallVector<CCValAssign, 16> RVLocs;
  MaxisCCState CCInfo(CallConv, IsVarArg, DAG.getMachineFunction(), RVLocs,
                     *DAG.getContext());

  const ExternalSymbolSDNode *ES =
      dyn_cast_or_null<const ExternalSymbolSDNode>(CLI.Callee.getNode());
  CCInfo.AnalyzeCallResult(Ins, RetCC_Maxis, CLI.RetTy,
                           ES ? ES->getSymbol() : nullptr);

  // Copy all of the result registers out of their specified physreg.
  for (unsigned i = 0; i != RVLocs.size(); ++i) {
    CCValAssign &VA = RVLocs[i];
    assert(VA.isRegLoc() && "Can only return in registers!");

    SDValue Val = DAG.getCopyFromReg(Chain, DL, RVLocs[i].getLocReg(),
                                     RVLocs[i].getLocVT(), InFlag);
    Chain = Val.getValue(1);
    InFlag = Val.getValue(2);

    if (VA.isUpperBitsInLoc()) {
      unsigned ValSizeInBits = Ins[i].ArgVT.getSizeInBits();
      unsigned LocSizeInBits = VA.getLocVT().getSizeInBits();
      unsigned Shift =
          VA.getLocInfo() == CCValAssign::ZExtUpper ? ISD::SRL : ISD::SRA;
      Val = DAG.getNode(
          Shift, DL, VA.getLocVT(), Val,
          DAG.getConstant(LocSizeInBits - ValSizeInBits, DL, VA.getLocVT()));
    }

    switch (VA.getLocInfo()) {
    default:
      llvm_unreachable("Unknown loc info!");
    case CCValAssign::Full:
      break;
    case CCValAssign::BCvt:
      Val = DAG.getNode(ISD::BITCAST, DL, VA.getValVT(), Val);
      break;
    case CCValAssign::AExt:
    case CCValAssign::AExtUpper:
      Val = DAG.getNode(ISD::TRUNCATE, DL, VA.getValVT(), Val);
      break;
    case CCValAssign::ZExt:
    case CCValAssign::ZExtUpper:
      Val = DAG.getNode(ISD::AssertZext, DL, VA.getLocVT(), Val,
                        DAG.getValueType(VA.getValVT()));
      Val = DAG.getNode(ISD::TRUNCATE, DL, VA.getValVT(), Val);
      break;
    case CCValAssign::SExt:
    case CCValAssign::SExtUpper:
      Val = DAG.getNode(ISD::AssertSext, DL, VA.getLocVT(), Val,
                        DAG.getValueType(VA.getValVT()));
      Val = DAG.getNode(ISD::TRUNCATE, DL, VA.getValVT(), Val);
      break;
    }

    InVals.push_back(Val);
  }

  return Chain;
}

static SDValue UnpackFromArgumentSlot(SDValue Val, const CCValAssign &VA,
                                      EVT ArgVT, const SDLoc &DL,
                                      SelectionDAG &DAG) {
  MVT LocVT = VA.getLocVT();
  EVT ValVT = VA.getValVT();

  // Shift into the upper bits if necessary.
  switch (VA.getLocInfo()) {
  default:
    break;
  case CCValAssign::AExtUpper:
  case CCValAssign::SExtUpper:
  case CCValAssign::ZExtUpper: {
    unsigned ValSizeInBits = ArgVT.getSizeInBits();
    unsigned LocSizeInBits = VA.getLocVT().getSizeInBits();
    unsigned Opcode =
        VA.getLocInfo() == CCValAssign::ZExtUpper ? ISD::SRL : ISD::SRA;
    Val = DAG.getNode(
        Opcode, DL, VA.getLocVT(), Val,
        DAG.getConstant(LocSizeInBits - ValSizeInBits, DL, VA.getLocVT()));
    break;
  }
  }

  // If this is an value smaller than the argument slot size (32-bit for O32,
  // 64-bit for N32/N64), it has been promoted in some way to the argument slot
  // size. Extract the value and insert any appropriate assertions regarding
  // sign/zero extension.
  switch (VA.getLocInfo()) {
  default:
    llvm_unreachable("Unknown loc info!");
  case CCValAssign::Full:
    break;
  case CCValAssign::AExtUpper:
  case CCValAssign::AExt:
    Val = DAG.getNode(ISD::TRUNCATE, DL, ValVT, Val);
    break;
  case CCValAssign::SExtUpper:
  case CCValAssign::SExt:
    Val = DAG.getNode(ISD::AssertSext, DL, LocVT, Val, DAG.getValueType(ValVT));
    Val = DAG.getNode(ISD::TRUNCATE, DL, ValVT, Val);
    break;
  case CCValAssign::ZExtUpper:
  case CCValAssign::ZExt:
    Val = DAG.getNode(ISD::AssertZext, DL, LocVT, Val, DAG.getValueType(ValVT));
    Val = DAG.getNode(ISD::TRUNCATE, DL, ValVT, Val);
    break;
  case CCValAssign::BCvt:
    Val = DAG.getNode(ISD::BITCAST, DL, ValVT, Val);
    break;
  }

  return Val;
}

//===----------------------------------------------------------------------===//
//             Formal Arguments Calling Convention Implementation
//===----------------------------------------------------------------------===//
/// LowerFormalArguments - transform physical registers into virtual registers
/// and generate load operations for arguments places on the stack.
SDValue MaxisTargetLowering::LowerFormalArguments(
    SDValue Chain, CallingConv::ID CallConv, bool IsVarArg,
    const SmallVectorImpl<ISD::InputArg> &Ins, const SDLoc &DL,
    SelectionDAG &DAG, SmallVectorImpl<SDValue> &InVals) const {
  MachineFunction &MF = DAG.getMachineFunction();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  MaxisFunctionInfo *MaxisFI = MF.getInfo<MaxisFunctionInfo>();

  MaxisFI->setVarArgsFrameIndex(0);

  // Used with vargs to acumulate store chains.
  std::vector<SDValue> OutChains;

  // Assign locations to all of the incoming arguments.
  SmallVector<CCValAssign, 16> ArgLocs;
  MaxisCCState CCInfo(CallConv, IsVarArg, DAG.getMachineFunction(), ArgLocs,
                     *DAG.getContext());
  CCInfo.AllocateStack(ABI.GetCalleeAllocdArgSizeInBytes(CallConv), 1);
  const Function &Func = DAG.getMachineFunction().getFunction();
  Function::const_arg_iterator FuncArg = Func.arg_begin();

  if (Func.hasFnAttribute("interrupt") && !Func.arg_empty())
    report_fatal_error(
        "Functions with the interrupt attribute cannot have arguments!");

  CCInfo.AnalyzeFormalArguments(Ins, CC_Maxis_FixedArg);
  MaxisFI->setFormalArgInfo(CCInfo.getNextStackOffset(),
                           CCInfo.getInRegsParamsCount() > 0);

  unsigned CurArgIdx = 0;
  CCInfo.rewindByValRegsInfo();

  for (unsigned i = 0, e = ArgLocs.size(); i != e; ++i) {
    CCValAssign &VA = ArgLocs[i];
    if (Ins[i].isOrigArg()) {
      std::advance(FuncArg, Ins[i].getOrigArgIndex() - CurArgIdx);
      CurArgIdx = Ins[i].getOrigArgIndex();
    }
    EVT ValVT = VA.getValVT();
    ISD::ArgFlagsTy Flags = Ins[i].Flags;
    bool IsRegLoc = VA.isRegLoc();

    if (Flags.isByVal()) {
      assert(Ins[i].isOrigArg() && "Byval arguments cannot be implicit");
      unsigned FirstByValReg, LastByValReg;
      unsigned ByValIdx = CCInfo.getInRegsParamsProcessed();
      CCInfo.getInRegsParamInfo(ByValIdx, FirstByValReg, LastByValReg);

      assert(Flags.getByValSize() &&
             "ByVal args of size 0 should have been ignored by front-end.");
      assert(ByValIdx < CCInfo.getInRegsParamsCount());
      copyByValRegs(Chain, DL, OutChains, DAG, Flags, InVals, &*FuncArg,
                    FirstByValReg, LastByValReg, VA, CCInfo);
      CCInfo.nextInRegsParam();
      continue;
    }

    // Arguments stored on registers
    if (IsRegLoc) {
      MVT RegVT = VA.getLocVT();
      unsigned ArgReg = VA.getLocReg();
      const TargetRegisterClass *RC = getRegClassFor(RegVT);

      // Transform the arguments stored on
      // physical registers into virtual ones
      unsigned Reg = addLiveIn(DAG.getMachineFunction(), ArgReg, RC);
      SDValue ArgValue = DAG.getCopyFromReg(Chain, DL, Reg, RegVT);

      ArgValue = UnpackFromArgumentSlot(ArgValue, VA, Ins[i].ArgVT, DL, DAG);

      // Handle floating point arguments passed in integer registers and
      // long double arguments passed in floating point registers.
      if ((RegVT == MVT::i32 && ValVT == MVT::f32) ||
          (RegVT == MVT::i64 && ValVT == MVT::f64) ||
          (RegVT == MVT::f64 && ValVT == MVT::i64))
        ArgValue = DAG.getNode(ISD::BITCAST, DL, ValVT, ArgValue);
      else if (ABI.IsO32() && RegVT == MVT::i32 &&
               ValVT == MVT::f64) {
        unsigned Reg2 = addLiveIn(DAG.getMachineFunction(),
                                  getNextIntArgReg(ArgReg), RC);
        SDValue ArgValue2 = DAG.getCopyFromReg(Chain, DL, Reg2, RegVT);
        if (!Subtarget.isLittle())
          std::swap(ArgValue, ArgValue2);
        ArgValue = DAG.getNode(MaxisISD::BuildPairF64, DL, MVT::f64,
                               ArgValue, ArgValue2);
      }

      InVals.push_back(ArgValue);
    } else { // VA.isRegLoc()
      MVT LocVT = VA.getLocVT();

      if (ABI.IsO32()) {
        // We ought to be able to use LocVT directly but O32 sets it to i32
        // when allocating floating point values to integer registers.
        // This shouldn't influence how we load the value into registers unless
        // we are targeting softfloat.
        if (VA.getValVT().isFloatingPoint() && !Subtarget.useSoftFloat())
          LocVT = VA.getValVT();
      }

      // sanity check
      assert(VA.isMemLoc());

      // The stack pointer offset is relative to the caller stack frame.
      int FI = MFI.CreateFixedObject(LocVT.getSizeInBits() / 8,
                                     VA.getLocMemOffset(), true);

      // Create load nodes to retrieve arguments from the stack
      SDValue FIN = DAG.getFrameIndex(FI, getPointerTy(DAG.getDataLayout()));
      SDValue ArgValue = DAG.getLoad(
          LocVT, DL, Chain, FIN,
          MachinePointerInfo::getFixedStack(DAG.getMachineFunction(), FI));
      OutChains.push_back(ArgValue.getValue(1));

      ArgValue = UnpackFromArgumentSlot(ArgValue, VA, Ins[i].ArgVT, DL, DAG);

      InVals.push_back(ArgValue);
    }
  }

  for (unsigned i = 0, e = ArgLocs.size(); i != e; ++i) {
    // The maxis ABIs for returning structs by value requires that we copy
    // the sret argument into $v0 for the return. Save the argument into
    // a virtual register so that we can access it from the return points.
    if (Ins[i].Flags.isSRet()) {
      unsigned Reg = MaxisFI->getSRetReturnReg();
      if (!Reg) {
        Reg = MF.getRegInfo().createVirtualRegister(
            getRegClassFor(ABI.IsN64() ? MVT::i64 : MVT::i32));
        MaxisFI->setSRetReturnReg(Reg);
      }
      SDValue Copy = DAG.getCopyToReg(DAG.getEntryNode(), DL, Reg, InVals[i]);
      Chain = DAG.getNode(ISD::TokenFactor, DL, MVT::Other, Copy, Chain);
      break;
    }
  }

  if (IsVarArg)
    writeVarArgRegs(OutChains, Chain, DL, DAG, CCInfo);

  // All stores are grouped in one node to allow the matching between
  // the size of Ins and InVals. This only happens when on varg functions
  if (!OutChains.empty()) {
    OutChains.push_back(Chain);
    Chain = DAG.getNode(ISD::TokenFactor, DL, MVT::Other, OutChains);
  }

  return Chain;
}

//===----------------------------------------------------------------------===//
//               Return Value Calling Convention Implementation
//===----------------------------------------------------------------------===//

bool
MaxisTargetLowering::CanLowerReturn(CallingConv::ID CallConv,
                                   MachineFunction &MF, bool IsVarArg,
                                   const SmallVectorImpl<ISD::OutputArg> &Outs,
                                   LLVMContext &Context) const {
  SmallVector<CCValAssign, 16> RVLocs;
  MaxisCCState CCInfo(CallConv, IsVarArg, MF, RVLocs, Context);
  return CCInfo.CheckReturn(Outs, RetCC_Maxis);
}

bool
MaxisTargetLowering::shouldSignExtendTypeInLibCall(EVT Type, bool IsSigned) const {
  if ((ABI.IsN32() || ABI.IsN64()) && Type == MVT::i32)
      return true;

  return IsSigned;
}

SDValue
MaxisTargetLowering::LowerInterruptReturn(SmallVectorImpl<SDValue> &RetOps,
                                         const SDLoc &DL,
                                         SelectionDAG &DAG) const {
  MachineFunction &MF = DAG.getMachineFunction();
  MaxisFunctionInfo *MaxisFI = MF.getInfo<MaxisFunctionInfo>();

  MaxisFI->setISR();

  return DAG.getNode(MaxisISD::ERet, DL, MVT::Other, RetOps);
}

SDValue
MaxisTargetLowering::LowerReturn(SDValue Chain, CallingConv::ID CallConv,
                                bool IsVarArg,
                                const SmallVectorImpl<ISD::OutputArg> &Outs,
                                const SmallVectorImpl<SDValue> &OutVals,
                                const SDLoc &DL, SelectionDAG &DAG) const {
  // CCValAssign - represent the assignment of
  // the return value to a location
  SmallVector<CCValAssign, 16> RVLocs;
  MachineFunction &MF = DAG.getMachineFunction();

  // CCState - Info about the registers and stack slot.
  MaxisCCState CCInfo(CallConv, IsVarArg, MF, RVLocs, *DAG.getContext());

  // Analyze return values.
  CCInfo.AnalyzeReturn(Outs, RetCC_Maxis);

  SDValue Flag;
  SmallVector<SDValue, 4> RetOps(1, Chain);

  // Copy the result values into the output registers.
  for (unsigned i = 0; i != RVLocs.size(); ++i) {
    SDValue Val = OutVals[i];
    CCValAssign &VA = RVLocs[i];
    assert(VA.isRegLoc() && "Can only return in registers!");
    bool UseUpperBits = false;

    switch (VA.getLocInfo()) {
    default:
      llvm_unreachable("Unknown loc info!");
    case CCValAssign::Full:
      break;
    case CCValAssign::BCvt:
      Val = DAG.getNode(ISD::BITCAST, DL, VA.getLocVT(), Val);
      break;
    case CCValAssign::AExtUpper:
      UseUpperBits = true;
      LLVM_FALLTHROUGH;
    case CCValAssign::AExt:
      Val = DAG.getNode(ISD::ANY_EXTEND, DL, VA.getLocVT(), Val);
      break;
    case CCValAssign::ZExtUpper:
      UseUpperBits = true;
      LLVM_FALLTHROUGH;
    case CCValAssign::ZExt:
      Val = DAG.getNode(ISD::ZERO_EXTEND, DL, VA.getLocVT(), Val);
      break;
    case CCValAssign::SExtUpper:
      UseUpperBits = true;
      LLVM_FALLTHROUGH;
    case CCValAssign::SExt:
      Val = DAG.getNode(ISD::SIGN_EXTEND, DL, VA.getLocVT(), Val);
      break;
    }

    if (UseUpperBits) {
      unsigned ValSizeInBits = Outs[i].ArgVT.getSizeInBits();
      unsigned LocSizeInBits = VA.getLocVT().getSizeInBits();
      Val = DAG.getNode(
          ISD::SHL, DL, VA.getLocVT(), Val,
          DAG.getConstant(LocSizeInBits - ValSizeInBits, DL, VA.getLocVT()));
    }

    Chain = DAG.getCopyToReg(Chain, DL, VA.getLocReg(), Val, Flag);

    // Guarantee that all emitted copies are stuck together with flags.
    Flag = Chain.getValue(1);
    RetOps.push_back(DAG.getRegister(VA.getLocReg(), VA.getLocVT()));
  }

  // The maxis ABIs for returning structs by value requires that we copy
  // the sret argument into $v0 for the return. We saved the argument into
  // a virtual register in the entry block, so now we copy the value out
  // and into $v0.
  if (MF.getFunction().hasStructRetAttr()) {
    MaxisFunctionInfo *MaxisFI = MF.getInfo<MaxisFunctionInfo>();
    unsigned Reg = MaxisFI->getSRetReturnReg();

    if (!Reg)
      llvm_unreachable("sret virtual register not created in the entry block");
    SDValue Val =
        DAG.getCopyFromReg(Chain, DL, Reg, getPointerTy(DAG.getDataLayout()));
    unsigned V0 = ABI.IsN64() ? Maxis::V0_64 : Maxis::V0;

    Chain = DAG.getCopyToReg(Chain, DL, V0, Val, Flag);
    Flag = Chain.getValue(1);
    RetOps.push_back(DAG.getRegister(V0, getPointerTy(DAG.getDataLayout())));
  }

  RetOps[0] = Chain;  // Update chain.

  // Add the flag if we have it.
  if (Flag.getNode())
    RetOps.push_back(Flag);

  // ISRs must use "eret".
  if (DAG.getMachineFunction().getFunction().hasFnAttribute("interrupt"))
    return LowerInterruptReturn(RetOps, DL, DAG);

  // Standard return on Maxis is a "jr $ra"
  return DAG.getNode(MaxisISD::Ret, DL, MVT::Other, RetOps);
}

//===----------------------------------------------------------------------===//
//                           Maxis Inline Assembly Support
//===----------------------------------------------------------------------===//

/// getConstraintType - Given a constraint letter, return the type of
/// constraint it is for this target.
MaxisTargetLowering::ConstraintType
MaxisTargetLowering::getConstraintType(StringRef Constraint) const {
  // Maxis specific constraints
  // GCC config/maxis/constraints.md
  //
  // 'd' : An address register. Equivalent to r
  //       unless generating MAXIS16 code.
  // 'y' : Equivalent to r; retained for
  //       backwards compatibility.
  // 'c' : A register suitable for use in an indirect
  //       jump. This will always be $25 for -mabicalls.
  // 'l' : The lo register. 1 word storage.
  // 'x' : The hilo register pair. Double word storage.
  if (Constraint.size() == 1) {
    switch (Constraint[0]) {
      default : break;
      case 'd':
      case 'y':
      case 'f':
      case 'c':
      case 'l':
      case 'x':
        return C_RegisterClass;
      case 'R':
        return C_Memory;
    }
  }

  if (Constraint == "ZC")
    return C_Memory;

  return TargetLowering::getConstraintType(Constraint);
}

/// Examine constraint type and operand type and determine a weight value.
/// This object must already have been set up with the operand type
/// and the current alternative constraint selected.
TargetLowering::ConstraintWeight
MaxisTargetLowering::getSingleConstraintMatchWeight(
    AsmOperandInfo &info, const char *constraint) const {
  ConstraintWeight weight = CW_Invalid;
  Value *CallOperandVal = info.CallOperandVal;
    // If we don't have a value, we can't do a match,
    // but allow it at the lowest weight.
  if (!CallOperandVal)
    return CW_Default;
  Type *type = CallOperandVal->getType();
  // Look at the constraint type.
  switch (*constraint) {
  default:
    weight = TargetLowering::getSingleConstraintMatchWeight(info, constraint);
    break;
  case 'd':
  case 'y':
    if (type->isIntegerTy())
      weight = CW_Register;
    break;
  case 'f': // FPU or MSA register
    if (Subtarget.hasMSA() && type->isVectorTy() &&
        cast<VectorType>(type)->getBitWidth() == 128)
      weight = CW_Register;
    else if (type->isFloatTy())
      weight = CW_Register;
    break;
  case 'c': // $25 for indirect jumps
  case 'l': // lo register
  case 'x': // hilo register pair
    if (type->isIntegerTy())
      weight = CW_SpecificReg;
    break;
  case 'I': // signed 16 bit immediate
  case 'J': // integer zero
  case 'K': // unsigned 16 bit immediate
  case 'L': // signed 32 bit immediate where lower 16 bits are 0
  case 'N': // immediate in the range of -65535 to -1 (inclusive)
  case 'O': // signed 15 bit immediate (+- 16383)
  case 'P': // immediate in the range of 65535 to 1 (inclusive)
    if (isa<ConstantInt>(CallOperandVal))
      weight = CW_Constant;
    break;
  case 'R':
    weight = CW_Memory;
    break;
  }
  return weight;
}

/// This is a helper function to parse a physical register string and split it
/// into non-numeric and numeric parts (Prefix and Reg). The first boolean flag
/// that is returned indicates whether parsing was successful. The second flag
/// is true if the numeric part exists.
static std::pair<bool, bool> parsePhysicalReg(StringRef C, StringRef &Prefix,
                                              unsigned long long &Reg) {
  if (C.front() != '{' || C.back() != '}')
    return std::make_pair(false, false);

  // Search for the first numeric character.
  StringRef::const_iterator I, B = C.begin() + 1, E = C.end() - 1;
  I = std::find_if(B, E, isdigit);

  Prefix = StringRef(B, I - B);

  // The second flag is set to false if no numeric characters were found.
  if (I == E)
    return std::make_pair(true, false);

  // Parse the numeric characters.
  return std::make_pair(!getAsUnsignedInteger(StringRef(I, E - I), 10, Reg),
                        true);
}

std::pair<unsigned, const TargetRegisterClass *> MaxisTargetLowering::
parseRegForInlineAsmConstraint(StringRef C, MVT VT) const {
  const TargetRegisterInfo *TRI =
      Subtarget.getRegisterInfo();
  const TargetRegisterClass *RC;
  StringRef Prefix;
  unsigned long long Reg;

  std::pair<bool, bool> R = parsePhysicalReg(C, Prefix, Reg);

  if (!R.first)
    return std::make_pair(0U, nullptr);

  if ((Prefix == "hi" || Prefix == "lo")) { // Parse hi/lo.
    // No numeric characters follow "hi" or "lo".
    if (R.second)
      return std::make_pair(0U, nullptr);

    RC = TRI->getRegClass(Prefix == "hi" ?
                          Maxis::HI32RegClassID : Maxis::LO32RegClassID);
    return std::make_pair(*(RC->begin()), RC);
  } else if (Prefix.startswith("$msa")) {
    // Parse $msa(ir|csr|access|save|modify|request|map|unmap)

    // No numeric characters follow the name.
    if (R.second)
      return std::make_pair(0U, nullptr);

    Reg = StringSwitch<unsigned long long>(Prefix)
              .Case("$msair", Maxis::MSAIR)
              .Case("$msacsr", Maxis::MSACSR)
              .Case("$msaaccess", Maxis::MSAAccess)
              .Case("$msasave", Maxis::MSASave)
              .Case("$msamodify", Maxis::MSAModify)
              .Case("$msarequest", Maxis::MSARequest)
              .Case("$msamap", Maxis::MSAMap)
              .Case("$msaunmap", Maxis::MSAUnmap)
              .Default(0);

    if (!Reg)
      return std::make_pair(0U, nullptr);

    RC = TRI->getRegClass(Maxis::MSACtrlRegClassID);
    return std::make_pair(Reg, RC);
  }

  if (!R.second)
    return std::make_pair(0U, nullptr);

  if (Prefix == "$f") { // Parse $f0-$f31.
    // If the size of FP registers is 64-bit or Reg is an even number, select
    // the 64-bit register class. Otherwise, select the 32-bit register class.
    if (VT == MVT::Other)
      VT = (Subtarget.isFP64bit() || !(Reg % 2)) ? MVT::f64 : MVT::f32;

    RC = getRegClassFor(VT);

    if (RC == &Maxis::AFGR64RegClass) {
      assert(Reg % 2 == 0);
      Reg >>= 1;
    }
  } else if (Prefix == "$fcc") // Parse $fcc0-$fcc7.
    RC = TRI->getRegClass(Maxis::FCCRegClassID);
  else if (Prefix == "$w") { // Parse $w0-$w31.
    RC = getRegClassFor((VT == MVT::Other) ? MVT::v16i8 : VT);
  } else { // Parse $0-$31.
    assert(Prefix == "$");
    RC = getRegClassFor((VT == MVT::Other) ? MVT::i32 : VT);
  }

  assert(Reg < RC->getNumRegs());
  return std::make_pair(*(RC->begin() + Reg), RC);
}

/// Given a register class constraint, like 'r', if this corresponds directly
/// to an LLVM register class, return a register of 0 and the register class
/// pointer.
std::pair<unsigned, const TargetRegisterClass *>
MaxisTargetLowering::getRegForInlineAsmConstraint(const TargetRegisterInfo *TRI,
                                                 StringRef Constraint,
                                                 MVT VT) const {
  if (Constraint.size() == 1) {
    switch (Constraint[0]) {
    case 'd': // Address register. Same as 'r' unless generating MAXIS16 code.
    case 'y': // Same as 'r'. Exists for compatibility.
    case 'r':
      if (VT == MVT::i32 || VT == MVT::i16 || VT == MVT::i8) {
        if (Subtarget.inMaxis16Mode())
          return std::make_pair(0U, &Maxis::CPU16RegsRegClass);
        return std::make_pair(0U, &Maxis::GPR32RegClass);
      }
      if (VT == MVT::i64 && !Subtarget.isGP64bit())
        return std::make_pair(0U, &Maxis::GPR32RegClass);
      if (VT == MVT::i64 && Subtarget.isGP64bit())
        return std::make_pair(0U, &Maxis::GPR64RegClass);
      // This will generate an error message
      return std::make_pair(0U, nullptr);
    case 'f': // FPU or MSA register
      if (VT == MVT::v16i8)
        return std::make_pair(0U, &Maxis::MSA128BRegClass);
      else if (VT == MVT::v8i16 || VT == MVT::v8f16)
        return std::make_pair(0U, &Maxis::MSA128HRegClass);
      else if (VT == MVT::v4i32 || VT == MVT::v4f32)
        return std::make_pair(0U, &Maxis::MSA128WRegClass);
      else if (VT == MVT::v2i64 || VT == MVT::v2f64)
        return std::make_pair(0U, &Maxis::MSA128DRegClass);
      else if (VT == MVT::f32)
        return std::make_pair(0U, &Maxis::FGR32RegClass);
      else if ((VT == MVT::f64) && (!Subtarget.isSingleFloat())) {
        if (Subtarget.isFP64bit())
          return std::make_pair(0U, &Maxis::FGR64RegClass);
        return std::make_pair(0U, &Maxis::AFGR64RegClass);
      }
      break;
    case 'c': // register suitable for indirect jump
      if (VT == MVT::i32)
        return std::make_pair((unsigned)Maxis::T9, &Maxis::GPR32RegClass);
      if (VT == MVT::i64)
        return std::make_pair((unsigned)Maxis::T9_64, &Maxis::GPR64RegClass);
      // This will generate an error message
      return std::make_pair(0U, nullptr);
    case 'l': // use the `lo` register to store values
              // that are no bigger than a word
      if (VT == MVT::i32)
        return std::make_pair((unsigned)Maxis::LO0, &Maxis::LO32RegClass);
      return std::make_pair((unsigned)Maxis::LO0_64, &Maxis::LO64RegClass);
    case 'x': // use the concatenated `hi` and `lo` registers
              // to store doubleword values
      // Fixme: Not triggering the use of both hi and low
      // This will generate an error message
      return std::make_pair(0U, nullptr);
    }
  }

  std::pair<unsigned, const TargetRegisterClass *> R;
  R = parseRegForInlineAsmConstraint(Constraint, VT);

  if (R.second)
    return R;

  return TargetLowering::getRegForInlineAsmConstraint(TRI, Constraint, VT);
}

/// LowerAsmOperandForConstraint - Lower the specified operand into the Ops
/// vector.  If it is invalid, don't add anything to Ops.
void MaxisTargetLowering::LowerAsmOperandForConstraint(SDValue Op,
                                                     std::string &Constraint,
                                                     std::vector<SDValue>&Ops,
                                                     SelectionDAG &DAG) const {
  SDLoc DL(Op);
  SDValue Result;

  // Only support length 1 constraints for now.
  if (Constraint.length() > 1) return;

  char ConstraintLetter = Constraint[0];
  switch (ConstraintLetter) {
  default: break; // This will fall through to the generic implementation
  case 'I': // Signed 16 bit constant
    // If this fails, the parent routine will give an error
    if (ConstantSDNode *C = dyn_cast<ConstantSDNode>(Op)) {
      EVT Type = Op.getValueType();
      int64_t Val = C->getSExtValue();
      if (isInt<16>(Val)) {
        Result = DAG.getTargetConstant(Val, DL, Type);
        break;
      }
    }
    return;
  case 'J': // integer zero
    if (ConstantSDNode *C = dyn_cast<ConstantSDNode>(Op)) {
      EVT Type = Op.getValueType();
      int64_t Val = C->getZExtValue();
      if (Val == 0) {
        Result = DAG.getTargetConstant(0, DL, Type);
        break;
      }
    }
    return;
  case 'K': // unsigned 16 bit immediate
    if (ConstantSDNode *C = dyn_cast<ConstantSDNode>(Op)) {
      EVT Type = Op.getValueType();
      uint64_t Val = (uint64_t)C->getZExtValue();
      if (isUInt<16>(Val)) {
        Result = DAG.getTargetConstant(Val, DL, Type);
        break;
      }
    }
    return;
  case 'L': // signed 32 bit immediate where lower 16 bits are 0
    if (ConstantSDNode *C = dyn_cast<ConstantSDNode>(Op)) {
      EVT Type = Op.getValueType();
      int64_t Val = C->getSExtValue();
      if ((isInt<32>(Val)) && ((Val & 0xffff) == 0)){
        Result = DAG.getTargetConstant(Val, DL, Type);
        break;
      }
    }
    return;
  case 'N': // immediate in the range of -65535 to -1 (inclusive)
    if (ConstantSDNode *C = dyn_cast<ConstantSDNode>(Op)) {
      EVT Type = Op.getValueType();
      int64_t Val = C->getSExtValue();
      if ((Val >= -65535) && (Val <= -1)) {
        Result = DAG.getTargetConstant(Val, DL, Type);
        break;
      }
    }
    return;
  case 'O': // signed 15 bit immediate
    if (ConstantSDNode *C = dyn_cast<ConstantSDNode>(Op)) {
      EVT Type = Op.getValueType();
      int64_t Val = C->getSExtValue();
      if ((isInt<15>(Val))) {
        Result = DAG.getTargetConstant(Val, DL, Type);
        break;
      }
    }
    return;
  case 'P': // immediate in the range of 1 to 65535 (inclusive)
    if (ConstantSDNode *C = dyn_cast<ConstantSDNode>(Op)) {
      EVT Type = Op.getValueType();
      int64_t Val = C->getSExtValue();
      if ((Val <= 65535) && (Val >= 1)) {
        Result = DAG.getTargetConstant(Val, DL, Type);
        break;
      }
    }
    return;
  }

  if (Result.getNode()) {
    Ops.push_back(Result);
    return;
  }

  TargetLowering::LowerAsmOperandForConstraint(Op, Constraint, Ops, DAG);
}

bool MaxisTargetLowering::isLegalAddressingMode(const DataLayout &DL,
                                               const AddrMode &AM, Type *Ty,
                                               unsigned AS, Instruction *I) const {
  // No global is ever allowed as a base.
  if (AM.BaseGV)
    return false;

  switch (AM.Scale) {
  case 0: // "r+i" or just "i", depending on HasBaseReg.
    break;
  case 1:
    if (!AM.HasBaseReg) // allow "r+i".
      break;
    return false; // disallow "r+r" or "r+r+i".
  default:
    return false;
  }

  return true;
}

bool
MaxisTargetLowering::isOffsetFoldingLegal(const GlobalAddressSDNode *GA) const {
  // The Maxis target isn't yet aware of offsets.
  return false;
}

EVT MaxisTargetLowering::getOptimalMemOpType(uint64_t Size, unsigned DstAlign,
                                            unsigned SrcAlign,
                                            bool IsMemset, bool ZeroMemset,
                                            bool MemcpyStrSrc,
                                            MachineFunction &MF) const {
  if (Subtarget.hasMaxis64())
    return MVT::i64;

  return MVT::i32;
}

bool MaxisTargetLowering::isFPImmLegal(const APFloat &Imm, EVT VT) const {
  if (VT != MVT::f32 && VT != MVT::f64)
    return false;
  if (Imm.isNegZero())
    return false;
  return Imm.isZero();
}

unsigned MaxisTargetLowering::getJumpTableEncoding() const {

  // FIXME: For space reasons this should be: EK_GPRel32BlockAddress.
  if (ABI.IsN64() && isPositionIndependent())
    return MachineJumpTableInfo::EK_GPRel64BlockAddress;

  return TargetLowering::getJumpTableEncoding();
}

bool MaxisTargetLowering::useSoftFloat() const {
  return Subtarget.useSoftFloat();
}

void MaxisTargetLowering::copyByValRegs(
    SDValue Chain, const SDLoc &DL, std::vector<SDValue> &OutChains,
    SelectionDAG &DAG, const ISD::ArgFlagsTy &Flags,
    SmallVectorImpl<SDValue> &InVals, const Argument *FuncArg,
    unsigned FirstReg, unsigned LastReg, const CCValAssign &VA,
    MaxisCCState &State) const {
  MachineFunction &MF = DAG.getMachineFunction();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  unsigned GPRSizeInBytes = Subtarget.getGPRSizeInBytes();
  unsigned NumRegs = LastReg - FirstReg;
  unsigned RegAreaSize = NumRegs * GPRSizeInBytes;
  unsigned FrameObjSize = std::max(Flags.getByValSize(), RegAreaSize);
  int FrameObjOffset;
  ArrayRef<MCPhysReg> ByValArgRegs = ABI.GetByValArgRegs();

  if (RegAreaSize)
    FrameObjOffset =
        (int)ABI.GetCalleeAllocdArgSizeInBytes(State.getCallingConv()) -
        (int)((ByValArgRegs.size() - FirstReg) * GPRSizeInBytes);
  else
    FrameObjOffset = VA.getLocMemOffset();

  // Create frame object.
  EVT PtrTy = getPointerTy(DAG.getDataLayout());
  int FI = MFI.CreateFixedObject(FrameObjSize, FrameObjOffset, true);
  SDValue FIN = DAG.getFrameIndex(FI, PtrTy);
  InVals.push_back(FIN);

  if (!NumRegs)
    return;

  // Copy arg registers.
  MVT RegTy = MVT::getIntegerVT(GPRSizeInBytes * 8);
  const TargetRegisterClass *RC = getRegClassFor(RegTy);

  for (unsigned I = 0; I < NumRegs; ++I) {
    unsigned ArgReg = ByValArgRegs[FirstReg + I];
    unsigned VReg = addLiveIn(MF, ArgReg, RC);
    unsigned Offset = I * GPRSizeInBytes;
    SDValue StorePtr = DAG.getNode(ISD::ADD, DL, PtrTy, FIN,
                                   DAG.getConstant(Offset, DL, PtrTy));
    SDValue Store = DAG.getStore(Chain, DL, DAG.getRegister(VReg, RegTy),
                                 StorePtr, MachinePointerInfo(FuncArg, Offset));
    OutChains.push_back(Store);
  }
}

// Copy byVal arg to registers and stack.
void MaxisTargetLowering::passByValArg(
    SDValue Chain, const SDLoc &DL,
    std::deque<std::pair<unsigned, SDValue>> &RegsToPass,
    SmallVectorImpl<SDValue> &MemOpChains, SDValue StackPtr,
    MachineFrameInfo &MFI, SelectionDAG &DAG, SDValue Arg, unsigned FirstReg,
    unsigned LastReg, const ISD::ArgFlagsTy &Flags, bool isLittle,
    const CCValAssign &VA) const {
  unsigned ByValSizeInBytes = Flags.getByValSize();
  unsigned OffsetInBytes = 0; // From beginning of struct
  unsigned RegSizeInBytes = Subtarget.getGPRSizeInBytes();
  unsigned Alignment = std::min(Flags.getByValAlign(), RegSizeInBytes);
  EVT PtrTy = getPointerTy(DAG.getDataLayout()),
      RegTy = MVT::getIntegerVT(RegSizeInBytes * 8);
  unsigned NumRegs = LastReg - FirstReg;

  if (NumRegs) {
    ArrayRef<MCPhysReg> ArgRegs = ABI.GetByValArgRegs();
    bool LeftoverBytes = (NumRegs * RegSizeInBytes > ByValSizeInBytes);
    unsigned I = 0;

    // Copy words to registers.
    for (; I < NumRegs - LeftoverBytes; ++I, OffsetInBytes += RegSizeInBytes) {
      SDValue LoadPtr = DAG.getNode(ISD::ADD, DL, PtrTy, Arg,
                                    DAG.getConstant(OffsetInBytes, DL, PtrTy));
      SDValue LoadVal = DAG.getLoad(RegTy, DL, Chain, LoadPtr,
                                    MachinePointerInfo(), Alignment);
      MemOpChains.push_back(LoadVal.getValue(1));
      unsigned ArgReg = ArgRegs[FirstReg + I];
      RegsToPass.push_back(std::make_pair(ArgReg, LoadVal));
    }

    // Return if the struct has been fully copied.
    if (ByValSizeInBytes == OffsetInBytes)
      return;

    // Copy the remainder of the byval argument with sub-word loads and shifts.
    if (LeftoverBytes) {
      SDValue Val;

      for (unsigned LoadSizeInBytes = RegSizeInBytes / 2, TotalBytesLoaded = 0;
           OffsetInBytes < ByValSizeInBytes; LoadSizeInBytes /= 2) {
        unsigned RemainingSizeInBytes = ByValSizeInBytes - OffsetInBytes;

        if (RemainingSizeInBytes < LoadSizeInBytes)
          continue;

        // Load subword.
        SDValue LoadPtr = DAG.getNode(ISD::ADD, DL, PtrTy, Arg,
                                      DAG.getConstant(OffsetInBytes, DL,
                                                      PtrTy));
        SDValue LoadVal = DAG.getExtLoad(
            ISD::ZEXTLOAD, DL, RegTy, Chain, LoadPtr, MachinePointerInfo(),
            MVT::getIntegerVT(LoadSizeInBytes * 8), Alignment);
        MemOpChains.push_back(LoadVal.getValue(1));

        // Shift the loaded value.
        unsigned Shamt;

        if (isLittle)
          Shamt = TotalBytesLoaded * 8;
        else
          Shamt = (RegSizeInBytes - (TotalBytesLoaded + LoadSizeInBytes)) * 8;

        SDValue Shift = DAG.getNode(ISD::SHL, DL, RegTy, LoadVal,
                                    DAG.getConstant(Shamt, DL, MVT::i32));

        if (Val.getNode())
          Val = DAG.getNode(ISD::OR, DL, RegTy, Val, Shift);
        else
          Val = Shift;

        OffsetInBytes += LoadSizeInBytes;
        TotalBytesLoaded += LoadSizeInBytes;
        Alignment = std::min(Alignment, LoadSizeInBytes);
      }

      unsigned ArgReg = ArgRegs[FirstReg + I];
      RegsToPass.push_back(std::make_pair(ArgReg, Val));
      return;
    }
  }

  // Copy remainder of byval arg to it with memcpy.
  unsigned MemCpySize = ByValSizeInBytes - OffsetInBytes;
  SDValue Src = DAG.getNode(ISD::ADD, DL, PtrTy, Arg,
                            DAG.getConstant(OffsetInBytes, DL, PtrTy));
  SDValue Dst = DAG.getNode(ISD::ADD, DL, PtrTy, StackPtr,
                            DAG.getIntPtrConstant(VA.getLocMemOffset(), DL));
  Chain = DAG.getMemcpy(Chain, DL, Dst, Src,
                        DAG.getConstant(MemCpySize, DL, PtrTy),
                        Alignment, /*isVolatile=*/false, /*AlwaysInline=*/false,
                        /*isTailCall=*/false,
                        MachinePointerInfo(), MachinePointerInfo());
  MemOpChains.push_back(Chain);
}

void MaxisTargetLowering::writeVarArgRegs(std::vector<SDValue> &OutChains,
                                         SDValue Chain, const SDLoc &DL,
                                         SelectionDAG &DAG,
                                         CCState &State) const {
  ArrayRef<MCPhysReg> ArgRegs = ABI.GetVarArgRegs();
  unsigned Idx = State.getFirstUnallocated(ArgRegs);
  unsigned RegSizeInBytes = Subtarget.getGPRSizeInBytes();
  MVT RegTy = MVT::getIntegerVT(RegSizeInBytes * 8);
  const TargetRegisterClass *RC = getRegClassFor(RegTy);
  MachineFunction &MF = DAG.getMachineFunction();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  MaxisFunctionInfo *MaxisFI = MF.getInfo<MaxisFunctionInfo>();

  // Offset of the first variable argument from stack pointer.
  int VaArgOffset;

  if (ArgRegs.size() == Idx)
    VaArgOffset = alignTo(State.getNextStackOffset(), RegSizeInBytes);
  else {
    VaArgOffset =
        (int)ABI.GetCalleeAllocdArgSizeInBytes(State.getCallingConv()) -
        (int)(RegSizeInBytes * (ArgRegs.size() - Idx));
  }

  // Record the frame index of the first variable argument
  // which is a value necessary to VASTART.
  int FI = MFI.CreateFixedObject(RegSizeInBytes, VaArgOffset, true);
  MaxisFI->setVarArgsFrameIndex(FI);

  // Copy the integer registers that have not been used for argument passing
  // to the argument register save area. For O32, the save area is allocated
  // in the caller's stack frame, while for N32/64, it is allocated in the
  // callee's stack frame.
  for (unsigned I = Idx; I < ArgRegs.size();
       ++I, VaArgOffset += RegSizeInBytes) {
    unsigned Reg = addLiveIn(MF, ArgRegs[I], RC);
    SDValue ArgValue = DAG.getCopyFromReg(Chain, DL, Reg, RegTy);
    FI = MFI.CreateFixedObject(RegSizeInBytes, VaArgOffset, true);
    SDValue PtrOff = DAG.getFrameIndex(FI, getPointerTy(DAG.getDataLayout()));
    SDValue Store =
        DAG.getStore(Chain, DL, ArgValue, PtrOff, MachinePointerInfo());
    cast<StoreSDNode>(Store.getNode())->getMemOperand()->setValue(
        (Value *)nullptr);
    OutChains.push_back(Store);
  }
}

void MaxisTargetLowering::HandleByVal(CCState *State, unsigned &Size,
                                     unsigned Align) const {
  const TargetFrameLowering *TFL = Subtarget.getFrameLowering();

  assert(Size && "Byval argument's size shouldn't be 0.");

  Align = std::min(Align, TFL->getStackAlignment());

  unsigned FirstReg = 0;
  unsigned NumRegs = 0;

  if (State->getCallingConv() != CallingConv::Fast) {
    unsigned RegSizeInBytes = Subtarget.getGPRSizeInBytes();
    ArrayRef<MCPhysReg> IntArgRegs = ABI.GetByValArgRegs();
    // FIXME: The O32 case actually describes no shadow registers.
    const MCPhysReg *ShadowRegs =
        ABI.IsO32() ? IntArgRegs.data() : Maxis64DPRegs;

    // We used to check the size as well but we can't do that anymore since
    // CCState::HandleByVal() rounds up the size after calling this function.
    assert(!(Align % RegSizeInBytes) &&
           "Byval argument's alignment should be a multiple of"
           "RegSizeInBytes.");

    FirstReg = State->getFirstUnallocated(IntArgRegs);

    // If Align > RegSizeInBytes, the first arg register must be even.
    // FIXME: This condition happens to do the right thing but it's not the
    //        right way to test it. We want to check that the stack frame offset
    //        of the register is aligned.
    if ((Align > RegSizeInBytes) && (FirstReg % 2)) {
      State->AllocateReg(IntArgRegs[FirstReg], ShadowRegs[FirstReg]);
      ++FirstReg;
    }

    // Mark the registers allocated.
    Size = alignTo(Size, RegSizeInBytes);
    for (unsigned I = FirstReg; Size > 0 && (I < IntArgRegs.size());
         Size -= RegSizeInBytes, ++I, ++NumRegs)
      State->AllocateReg(IntArgRegs[I], ShadowRegs[I]);
  }

  State->addInRegsParamInfo(FirstReg, FirstReg + NumRegs);
}

MachineBasicBlock *MaxisTargetLowering::emitPseudoSELECT(MachineInstr &MI,
                                                        MachineBasicBlock *BB,
                                                        bool isFPCmp,
                                                        unsigned Opc) const {
  assert(!(Subtarget.hasMaxis4() || Subtarget.hasMaxis32()) &&
         "Subtarget already supports SELECT nodes with the use of"
         "conditional-move instructions.");

  const TargetInstrInfo *TII =
      Subtarget.getInstrInfo();
  DebugLoc DL = MI.getDebugLoc();

  // To "insert" a SELECT instruction, we actually have to insert the
  // diamond control-flow pattern.  The incoming instruction knows the
  // destination vreg to set, the condition code register to branch on, the
  // true/false values to select between, and a branch opcode to use.
  const BasicBlock *LLVM_BB = BB->getBasicBlock();
  MachineFunction::iterator It = ++BB->getIterator();

  //  thisMBB:
  //  ...
  //   TrueVal = ...
  //   setcc r1, r2, r3
  //   bNE   r1, r0, copy1MBB
  //   fallthrough --> copy0MBB
  MachineBasicBlock *thisMBB  = BB;
  MachineFunction *F = BB->getParent();
  MachineBasicBlock *copy0MBB = F->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *sinkMBB  = F->CreateMachineBasicBlock(LLVM_BB);
  F->insert(It, copy0MBB);
  F->insert(It, sinkMBB);

  // Transfer the remainder of BB and its successor edges to sinkMBB.
  sinkMBB->splice(sinkMBB->begin(), BB,
                  std::next(MachineBasicBlock::iterator(MI)), BB->end());
  sinkMBB->transferSuccessorsAndUpdatePHIs(BB);

  // Next, add the true and fallthrough blocks as its successors.
  BB->addSuccessor(copy0MBB);
  BB->addSuccessor(sinkMBB);

  if (isFPCmp) {
    // bc1[tf] cc, sinkMBB
    BuildMI(BB, DL, TII->get(Opc))
        .addReg(MI.getOperand(1).getReg())
        .addMBB(sinkMBB);
  } else {
    // bne rs, $0, sinkMBB
    BuildMI(BB, DL, TII->get(Opc))
        .addReg(MI.getOperand(1).getReg())
        .addReg(Maxis::ZERO)
        .addMBB(sinkMBB);
  }

  //  copy0MBB:
  //   %FalseValue = ...
  //   # fallthrough to sinkMBB
  BB = copy0MBB;

  // Update machine-CFG edges
  BB->addSuccessor(sinkMBB);

  //  sinkMBB:
  //   %Result = phi [ %TrueValue, thisMBB ], [ %FalseValue, copy0MBB ]
  //  ...
  BB = sinkMBB;

  BuildMI(*BB, BB->begin(), DL, TII->get(Maxis::PHI), MI.getOperand(0).getReg())
      .addReg(MI.getOperand(2).getReg())
      .addMBB(thisMBB)
      .addReg(MI.getOperand(3).getReg())
      .addMBB(copy0MBB);

  MI.eraseFromParent(); // The pseudo instruction is gone now.

  return BB;
}

// FIXME? Maybe this could be a TableGen attribute on some registers and
// this table could be generated automatically from RegInfo.
unsigned MaxisTargetLowering::getRegisterByName(const char* RegName, EVT VT,
                                               SelectionDAG &DAG) const {
  // Named registers is expected to be fairly rare. For now, just support $28
  // since the linux kernel uses it.
  if (Subtarget.isGP64bit()) {
    unsigned Reg = StringSwitch<unsigned>(RegName)
                         .Case("$28", Maxis::GP_64)
                         .Default(0);
    if (Reg)
      return Reg;
  } else {
    unsigned Reg = StringSwitch<unsigned>(RegName)
                         .Case("$28", Maxis::GP)
                         .Default(0);
    if (Reg)
      return Reg;
  }
  report_fatal_error("Invalid register name global variable");
}
