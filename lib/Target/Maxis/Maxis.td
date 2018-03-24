//===-- Maxis.td - Describe the Maxis Target Machine ---------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
// This is the top level entry point for the Maxis target.
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Target-independent interfaces
//===----------------------------------------------------------------------===//

include "llvm/Target/Target.td"

// The overall idea of the PredicateControl class is to chop the Predicates list
// into subsets that are usually overridden independently. This allows
// subclasses to partially override the predicates of their superclasses without
// having to re-add all the existing predicates.
class PredicateControl {
  // Predicates for the encoding scheme in use such as HasStdEnc
  list<Predicate> EncodingPredicates = [];
  // Predicates for the GPR size such as IsGP64bit
  list<Predicate> GPRPredicates = [];
  // Predicates for the PTR size such as IsPTR64bit
  list<Predicate> PTRPredicates = [];
  // Predicates for the FGR size and layout such as IsFP64bit
  list<Predicate> FGRPredicates = [];
  // Predicates for the instruction group membership such as ISA's and ASE's
  list<Predicate> InsnPredicates = [];
  // Predicate for marking the instruction as usable in hard-float mode only.
  list<Predicate> HardFloatPredicate = [];
  // Predicates for anything else
  list<Predicate> AdditionalPredicates = [];
  list<Predicate> Predicates = !listconcat(EncodingPredicates,
                                           GPRPredicates,
                                           PTRPredicates,
                                           FGRPredicates,
                                           InsnPredicates,
                                           HardFloatPredicate,
                                           AdditionalPredicates);
}

// Like Requires<> but for the AdditionalPredicates list
class AdditionalRequires<list<Predicate> preds> {
  list<Predicate> AdditionalPredicates = preds;
}

//===----------------------------------------------------------------------===//
// Register File, Calling Conv, Instruction Descriptions
//===----------------------------------------------------------------------===//

include "MaxisRegisterInfo.td"
include "MaxisSchedule.td"
include "MaxisInstrInfo.td"
include "MaxisCallingConv.td"

// Avoid forward declaration issues.
include "MaxisScheduleP5600.td"
include "MaxisScheduleGeneric.td"

def MaxisInstrInfo : InstrInfo;

//===----------------------------------------------------------------------===//
// Maxis Subtarget features                                                    //
//===----------------------------------------------------------------------===//

def FeatureNoABICalls  : SubtargetFeature<"noabicalls", "NoABICalls", "true",
                                "Disable SVR4-style position-independent code">;
def FeaturePTR64Bit    : SubtargetFeature<"ptr64", "IsPTR64bit", "true",
                                "Pointers are 64-bit wide">;
def FeatureGP64Bit     : SubtargetFeature<"gp64", "IsGP64bit", "true",
                                "General Purpose Registers are 64-bit wide">;
def FeatureFP64Bit     : SubtargetFeature<"fp64", "IsFP64bit", "true",
                                "Support 64-bit FP registers">;
def FeatureFPXX        : SubtargetFeature<"fpxx", "IsFPXX", "true",
                                "Support for FPXX">;
def FeatureNaN2008     : SubtargetFeature<"nan2008", "IsNaN2008bit", "true",
                                "IEEE 754-2008 NaN encoding">;
def FeatureSingleFloat : SubtargetFeature<"single-float", "IsSingleFloat",
                                "true", "Only supports single precision float">;
def FeatureSoftFloat   : SubtargetFeature<"soft-float", "IsSoftFloat", "true",
                                "Does not support floating point instructions">;
def FeatureNoOddSPReg  : SubtargetFeature<"nooddspreg", "UseOddSPReg", "false",
                              "Disable odd numbered single-precision "
                              "registers">;
def FeatureVFPU        : SubtargetFeature<"vfpu", "HasVFPU",
                                "true", "Enable vector FPU instructions">;
def FeatureMaxis1       : SubtargetFeature<"maxis1", "MaxisArchVersion", "Maxis1",
                                "Maxis I ISA Support [highly experimental]">;
def FeatureMaxis2       : SubtargetFeature<"maxis2", "MaxisArchVersion", "Maxis2",
                                "Maxis II ISA Support [highly experimental]",
                                [FeatureMaxis1]>;
def FeatureMaxis3_32    : SubtargetFeature<"maxis3_32", "HasMaxis3_32", "true",
                                "Subset of MAXIS-III that is also in MAXIS32 "
                                "[highly experimental]">;
def FeatureMaxis3_32r2  : SubtargetFeature<"maxis3_32r2", "HasMaxis3_32r2", "true",
                                "Subset of MAXIS-III that is also in MAXIS32r2 "
                                "[highly experimental]">;
def FeatureMaxis3       : SubtargetFeature<"maxis3", "MaxisArchVersion", "Maxis3",
                                "MAXIS III ISA Support [highly experimental]",
                                [FeatureMaxis2, FeatureMaxis3_32,
                                 FeatureMaxis3_32r2, FeatureGP64Bit,
                                 FeatureFP64Bit]>;
def FeatureMaxis4_32    : SubtargetFeature<"maxis4_32", "HasMaxis4_32", "true",
                                "Subset of MAXIS-IV that is also in MAXIS32 "
                                "[highly experimental]">;
def FeatureMaxis4_32r2  : SubtargetFeature<"maxis4_32r2", "HasMaxis4_32r2", "true",
                                "Subset of MAXIS-IV that is also in MAXIS32r2 "
                                "[highly experimental]">;
def FeatureMaxis4       : SubtargetFeature<"maxis4", "MaxisArchVersion",
                                "Maxis4", "MAXIS IV ISA Support",
                                [FeatureMaxis3, FeatureMaxis4_32,
                                 FeatureMaxis4_32r2]>;
def FeatureMaxis5_32r2  : SubtargetFeature<"maxis5_32r2", "HasMaxis5_32r2", "true",
                                "Subset of MAXIS-V that is also in MAXIS32r2 "
                                "[highly experimental]">;
def FeatureMaxis5       : SubtargetFeature<"maxis5", "MaxisArchVersion", "Maxis5",
                                "MAXIS V ISA Support [highly experimental]",
                                [FeatureMaxis4, FeatureMaxis5_32r2]>;
def FeatureMaxis32      : SubtargetFeature<"maxis32", "MaxisArchVersion", "Maxis32",
                                "Maxis32 ISA Support",
                                [FeatureMaxis2, FeatureMaxis3_32,
                                 FeatureMaxis4_32]>;
def FeatureMaxis32r2    : SubtargetFeature<"maxis32r2", "MaxisArchVersion",
                                "Maxis32r2", "Maxis32r2 ISA Support",
                                [FeatureMaxis3_32r2, FeatureMaxis4_32r2,
                                 FeatureMaxis5_32r2, FeatureMaxis32]>;
def FeatureMaxis32r3    : SubtargetFeature<"maxis32r3", "MaxisArchVersion",
                                "Maxis32r3", "Maxis32r3 ISA Support",
                                [FeatureMaxis32r2]>;
def FeatureMaxis32r5    : SubtargetFeature<"maxis32r5", "MaxisArchVersion",
                                "Maxis32r5", "Maxis32r5 ISA Support",
                                [FeatureMaxis32r3]>;
def FeatureMaxis32r6    : SubtargetFeature<"maxis32r6", "MaxisArchVersion",
                                "Maxis32r6",
                                "Maxis32r6 ISA Support [experimental]",
                                [FeatureMaxis32r5, FeatureFP64Bit,
                                 FeatureNaN2008]>;
def FeatureMaxis64      : SubtargetFeature<"maxis64", "MaxisArchVersion",
                                "Maxis64", "Maxis64 ISA Support",
                                [FeatureMaxis5, FeatureMaxis32]>;
def FeatureMaxis64r2    : SubtargetFeature<"maxis64r2", "MaxisArchVersion",
                                "Maxis64r2", "Maxis64r2 ISA Support",
                                [FeatureMaxis64, FeatureMaxis32r2]>;
def FeatureMaxis64r3    : SubtargetFeature<"maxis64r3", "MaxisArchVersion",
                                "Maxis64r3", "Maxis64r3 ISA Support",
                                [FeatureMaxis64r2, FeatureMaxis32r3]>;
def FeatureMaxis64r5    : SubtargetFeature<"maxis64r5", "MaxisArchVersion",
                                "Maxis64r5", "Maxis64r5 ISA Support",
                                [FeatureMaxis64r3, FeatureMaxis32r5]>;
def FeatureMaxis64r6    : SubtargetFeature<"maxis64r6", "MaxisArchVersion",
                                "Maxis64r6",
                                "Maxis64r6 ISA Support [experimental]",
                                [FeatureMaxis32r6, FeatureMaxis64r5,
                                 FeatureNaN2008]>;
def FeatureSym32       : SubtargetFeature<"sym32", "HasSym32", "true",
                                          "Symbols are 32 bit on Maxis64">;

def FeatureMaxis16  : SubtargetFeature<"maxis16", "InMaxis16Mode", "true",
                                      "Maxis16 mode">;

def FeatureDSP : SubtargetFeature<"dsp", "HasDSP", "true", "Maxis DSP ASE">;
def FeatureDSPR2 : SubtargetFeature<"dspr2", "HasDSPR2", "true",
                                    "Maxis DSP-R2 ASE", [FeatureDSP]>;
def FeatureDSPR3
    : SubtargetFeature<"dspr3", "HasDSPR3", "true", "Maxis DSP-R3 ASE",
                       [ FeatureDSP, FeatureDSPR2 ]>;

def FeatureMSA : SubtargetFeature<"msa", "HasMSA", "true", "Maxis MSA ASE">;

def FeatureEVA : SubtargetFeature<"eva", "HasEVA", "true", "Maxis EVA ASE">;

def FeatureMicroMaxis  : SubtargetFeature<"micromaxis", "InMicroMaxisMode", "true",
                                         "microMaxis mode">;

def FeatureCnMaxis     : SubtargetFeature<"cnmaxis", "HasCnMaxis",
                                "true", "Octeon cnMAXIS Support",
                                [FeatureMaxis64r2]>;

def FeatureUseTCCInDIV : SubtargetFeature<
                               "use-tcc-in-div",
                               "UseTCCInDIV", "false",
                               "Force the assembler to use trapping">;

def FeatureMadd4 : SubtargetFeature<"nomadd4", "DisableMadd4", "true",
                                    "Disable 4-operand madd.fmt and related instructions">;

def FeatureMT : SubtargetFeature<"mt", "HasMT", "true", "Maxis MT ASE">;

def FeatureLongCalls : SubtargetFeature<"long-calls", "UseLongCalls", "true",
                                        "Disable use of the jal instruction">;

//===----------------------------------------------------------------------===//
// Maxis processors supported.
//===----------------------------------------------------------------------===//

def ImplP5600 : SubtargetFeature<"p5600", "ProcImpl",
                                 "MaxisSubtarget::CPU::P5600",
                                 "The P5600 Processor", [FeatureMaxis32r5]>;

class Proc<string Name, list<SubtargetFeature> Features>
 : ProcessorModel<Name, MaxisGenericModel, Features>;

def : Proc<"maxis1", [FeatureMaxis1]>;
def : Proc<"maxis2", [FeatureMaxis2]>;
def : Proc<"maxis32", [FeatureMaxis32]>;
def : Proc<"maxis32r2", [FeatureMaxis32r2]>;
def : Proc<"maxis32r3", [FeatureMaxis32r3]>;
def : Proc<"maxis32r5", [FeatureMaxis32r5]>;
def : Proc<"maxis32r6", [FeatureMaxis32r6]>;

def : Proc<"maxis3", [FeatureMaxis3]>;
def : Proc<"maxis4", [FeatureMaxis4]>;
def : Proc<"maxis5", [FeatureMaxis5]>;
def : Proc<"maxis64", [FeatureMaxis64]>;
def : Proc<"maxis64r2", [FeatureMaxis64r2]>;
def : Proc<"maxis64r3", [FeatureMaxis64r3]>;
def : Proc<"maxis64r5", [FeatureMaxis64r5]>;
def : Proc<"maxis64r6", [FeatureMaxis64r6]>;
def : Proc<"octeon", [FeatureMaxis64r2, FeatureCnMaxis]>;
def : ProcessorModel<"p5600", MaxisP5600Model, [ImplP5600]>;

def MaxisAsmParser : AsmParser {
  let ShouldEmitMatchRegisterName = 0;
}

def MaxisAsmParserVariant : AsmParserVariant {
  int Variant = 0;

  // Recognize hard coded registers.
  string RegisterPrefix = "$";
}

def Maxis : Target {
  let InstructionSet = MaxisInstrInfo;
  let AssemblyParsers = [MaxisAsmParser];
  let AssemblyParserVariants = [MaxisAsmParserVariant];
}
