//===- IntrinsicsMaxis.td - Defines Maxis intrinsics ---------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines all of the MAXIS-specific intrinsics.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// MAXIS DSP data types
def maxis_v2q15_ty: LLVMType<v2i16>;
def maxis_v4q7_ty: LLVMType<v4i8>;
def maxis_q31_ty: LLVMType<i32>;

let TargetPrefix = "maxis" in {  // All intrinsics start with "llvm.maxis.".

//===----------------------------------------------------------------------===//
// MAXIS DSP Rev 1

//===----------------------------------------------------------------------===//
// Addition/subtraction

def int_maxis_addu_qb : GCCBuiltin<"__builtin_maxis_addu_qb">,
  Intrinsic<[llvm_v4i8_ty], [llvm_v4i8_ty, llvm_v4i8_ty],
            [Commutative, IntrNoMem]>;
def int_maxis_addu_s_qb : GCCBuiltin<"__builtin_maxis_addu_s_qb">,
  Intrinsic<[llvm_v4i8_ty], [llvm_v4i8_ty, llvm_v4i8_ty],
            [Commutative, IntrNoMem]>;
def int_maxis_subu_qb : GCCBuiltin<"__builtin_maxis_subu_qb">,
  Intrinsic<[llvm_v4i8_ty], [llvm_v4i8_ty, llvm_v4i8_ty], [IntrNoMem]>;
def int_maxis_subu_s_qb : GCCBuiltin<"__builtin_maxis_subu_s_qb">,
  Intrinsic<[llvm_v4i8_ty], [llvm_v4i8_ty, llvm_v4i8_ty], [IntrNoMem]>;

def int_maxis_addq_ph : GCCBuiltin<"__builtin_maxis_addq_ph">,
  Intrinsic<[maxis_v2q15_ty], [maxis_v2q15_ty, maxis_v2q15_ty],
            [Commutative, IntrNoMem]>;
def int_maxis_addq_s_ph : GCCBuiltin<"__builtin_maxis_addq_s_ph">,
  Intrinsic<[maxis_v2q15_ty], [maxis_v2q15_ty, maxis_v2q15_ty],
            [Commutative, IntrNoMem]>;
def int_maxis_subq_ph : GCCBuiltin<"__builtin_maxis_subq_ph">,
  Intrinsic<[maxis_v2q15_ty], [maxis_v2q15_ty, maxis_v2q15_ty], [IntrNoMem]>;
def int_maxis_subq_s_ph : GCCBuiltin<"__builtin_maxis_subq_s_ph">,
  Intrinsic<[maxis_v2q15_ty], [maxis_v2q15_ty, maxis_v2q15_ty], [IntrNoMem]>;

def int_maxis_madd: GCCBuiltin<"__builtin_maxis_madd">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_i32_ty, llvm_i32_ty],
            [IntrNoMem, Commutative]>;
def int_maxis_maddu: GCCBuiltin<"__builtin_maxis_maddu">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_i32_ty, llvm_i32_ty],
            [IntrNoMem, Commutative]>;

def int_maxis_msub: GCCBuiltin<"__builtin_maxis_msub">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_i32_ty, llvm_i32_ty],
            [IntrNoMem]>;
def int_maxis_msubu: GCCBuiltin<"__builtin_maxis_msubu">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_i32_ty, llvm_i32_ty],
            [IntrNoMem]>;

def int_maxis_addq_s_w: GCCBuiltin<"__builtin_maxis_addq_s_w">,
  Intrinsic<[maxis_q31_ty], [maxis_q31_ty, maxis_q31_ty], [Commutative]>;
def int_maxis_subq_s_w: GCCBuiltin<"__builtin_maxis_subq_s_w">,
  Intrinsic<[maxis_q31_ty], [maxis_q31_ty, maxis_q31_ty], []>;

def int_maxis_addsc: GCCBuiltin<"__builtin_maxis_addsc">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty, llvm_i32_ty], [Commutative]>;
def int_maxis_addwc: GCCBuiltin<"__builtin_maxis_addwc">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty, llvm_i32_ty], [Commutative]>;

def int_maxis_modsub: GCCBuiltin<"__builtin_maxis_modsub">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_raddu_w_qb: GCCBuiltin<"__builtin_maxis_raddu_w_qb">,
  Intrinsic<[llvm_i32_ty], [llvm_v4i8_ty], [IntrNoMem]>;

//===----------------------------------------------------------------------===//
// Absolute value

def int_maxis_absq_s_ph: GCCBuiltin<"__builtin_maxis_absq_s_ph">,
  Intrinsic<[maxis_v2q15_ty], [maxis_v2q15_ty], []>;
def int_maxis_absq_s_w: GCCBuiltin<"__builtin_maxis_absq_s_w">,
  Intrinsic<[maxis_q31_ty], [maxis_q31_ty], []>;

//===----------------------------------------------------------------------===//
// Precision reduce/expand

def int_maxis_precrq_qb_ph: GCCBuiltin<"__builtin_maxis_precrq_qb_ph">,
  Intrinsic<[llvm_v4i8_ty], [maxis_v2q15_ty, maxis_v2q15_ty], [IntrNoMem]>;
def int_maxis_precrqu_s_qb_ph: GCCBuiltin<"__builtin_maxis_precrqu_s_qb_ph">,
  Intrinsic<[llvm_v4i8_ty], [maxis_v2q15_ty, maxis_v2q15_ty], []>;
def int_maxis_precrq_ph_w: GCCBuiltin<"__builtin_maxis_precrq_ph_w">,
  Intrinsic<[maxis_v2q15_ty], [maxis_q31_ty, maxis_q31_ty], [IntrNoMem]>;
def int_maxis_precrq_rs_ph_w: GCCBuiltin<"__builtin_maxis_precrq_rs_ph_w">,
  Intrinsic<[maxis_v2q15_ty], [maxis_q31_ty, maxis_q31_ty], []>;
def int_maxis_preceq_w_phl: GCCBuiltin<"__builtin_maxis_preceq_w_phl">,
  Intrinsic<[maxis_q31_ty], [maxis_v2q15_ty], [IntrNoMem]>;
def int_maxis_preceq_w_phr: GCCBuiltin<"__builtin_maxis_preceq_w_phr">,
  Intrinsic<[maxis_q31_ty], [maxis_v2q15_ty], [IntrNoMem]>;
def int_maxis_precequ_ph_qbl: GCCBuiltin<"__builtin_maxis_precequ_ph_qbl">,
  Intrinsic<[maxis_v2q15_ty], [llvm_v4i8_ty], [IntrNoMem]>;
def int_maxis_precequ_ph_qbr: GCCBuiltin<"__builtin_maxis_precequ_ph_qbr">,
  Intrinsic<[maxis_v2q15_ty], [llvm_v4i8_ty], [IntrNoMem]>;
def int_maxis_precequ_ph_qbla: GCCBuiltin<"__builtin_maxis_precequ_ph_qbla">,
  Intrinsic<[maxis_v2q15_ty], [llvm_v4i8_ty], [IntrNoMem]>;
def int_maxis_precequ_ph_qbra: GCCBuiltin<"__builtin_maxis_precequ_ph_qbra">,
  Intrinsic<[maxis_v2q15_ty], [llvm_v4i8_ty], [IntrNoMem]>;
def int_maxis_preceu_ph_qbl: GCCBuiltin<"__builtin_maxis_preceu_ph_qbl">,
  Intrinsic<[maxis_v2q15_ty], [llvm_v4i8_ty], [IntrNoMem]>;
def int_maxis_preceu_ph_qbr: GCCBuiltin<"__builtin_maxis_preceu_ph_qbr">,
  Intrinsic<[maxis_v2q15_ty], [llvm_v4i8_ty], [IntrNoMem]>;
def int_maxis_preceu_ph_qbla: GCCBuiltin<"__builtin_maxis_preceu_ph_qbla">,
  Intrinsic<[maxis_v2q15_ty], [llvm_v4i8_ty], [IntrNoMem]>;
def int_maxis_preceu_ph_qbra: GCCBuiltin<"__builtin_maxis_preceu_ph_qbra">,
  Intrinsic<[maxis_v2q15_ty], [llvm_v4i8_ty], [IntrNoMem]>;

//===----------------------------------------------------------------------===//
// Shift

def int_maxis_shll_qb: GCCBuiltin<"__builtin_maxis_shll_qb">,
  Intrinsic<[llvm_v4i8_ty], [llvm_v4i8_ty, llvm_i32_ty], []>;
def int_maxis_shrl_qb: GCCBuiltin<"__builtin_maxis_shrl_qb">,
  Intrinsic<[llvm_v4i8_ty], [llvm_v4i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_shll_ph: GCCBuiltin<"__builtin_maxis_shll_ph">,
  Intrinsic<[maxis_v2q15_ty], [maxis_v2q15_ty, llvm_i32_ty], []>;
def int_maxis_shll_s_ph: GCCBuiltin<"__builtin_maxis_shll_s_ph">,
  Intrinsic<[maxis_v2q15_ty], [maxis_v2q15_ty, llvm_i32_ty], []>;
def int_maxis_shra_ph: GCCBuiltin<"__builtin_maxis_shra_ph">,
  Intrinsic<[maxis_v2q15_ty], [maxis_v2q15_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_shra_r_ph: GCCBuiltin<"__builtin_maxis_shra_r_ph">,
  Intrinsic<[maxis_v2q15_ty], [maxis_v2q15_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_shll_s_w: GCCBuiltin<"__builtin_maxis_shll_s_w">,
  Intrinsic<[maxis_q31_ty], [maxis_q31_ty, llvm_i32_ty], []>;
def int_maxis_shra_r_w: GCCBuiltin<"__builtin_maxis_shra_r_w">,
  Intrinsic<[maxis_q31_ty], [maxis_q31_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_shilo: GCCBuiltin<"__builtin_maxis_shilo">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_i32_ty], [IntrNoMem]>;

//===----------------------------------------------------------------------===//
// Multiplication

def int_maxis_muleu_s_ph_qbl: GCCBuiltin<"__builtin_maxis_muleu_s_ph_qbl">,
  Intrinsic<[maxis_v2q15_ty], [llvm_v4i8_ty, maxis_v2q15_ty], []>;
def int_maxis_muleu_s_ph_qbr: GCCBuiltin<"__builtin_maxis_muleu_s_ph_qbr">,
  Intrinsic<[maxis_v2q15_ty], [llvm_v4i8_ty, maxis_v2q15_ty], []>;
def int_maxis_mulq_rs_ph: GCCBuiltin<"__builtin_maxis_mulq_rs_ph">,
  Intrinsic<[maxis_v2q15_ty], [maxis_v2q15_ty, maxis_v2q15_ty], [Commutative]>;
def int_maxis_muleq_s_w_phl: GCCBuiltin<"__builtin_maxis_muleq_s_w_phl">,
  Intrinsic<[maxis_q31_ty], [maxis_v2q15_ty, maxis_v2q15_ty], [Commutative]>;
def int_maxis_muleq_s_w_phr: GCCBuiltin<"__builtin_maxis_muleq_s_w_phr">,
  Intrinsic<[maxis_q31_ty], [maxis_v2q15_ty, maxis_v2q15_ty], [Commutative]>;
def int_maxis_mulsaq_s_w_ph: GCCBuiltin<"__builtin_maxis_mulsaq_s_w_ph">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, maxis_v2q15_ty, maxis_v2q15_ty], []>;
def int_maxis_maq_s_w_phl: GCCBuiltin<"__builtin_maxis_maq_s_w_phl">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, maxis_v2q15_ty, maxis_v2q15_ty], []>;
def int_maxis_maq_s_w_phr: GCCBuiltin<"__builtin_maxis_maq_s_w_phr">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, maxis_v2q15_ty, maxis_v2q15_ty], []>;
def int_maxis_maq_sa_w_phl: GCCBuiltin<"__builtin_maxis_maq_sa_w_phl">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, maxis_v2q15_ty, maxis_v2q15_ty], []>;
def int_maxis_maq_sa_w_phr: GCCBuiltin<"__builtin_maxis_maq_sa_w_phr">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, maxis_v2q15_ty, maxis_v2q15_ty], []>;
def int_maxis_mult: GCCBuiltin<"__builtin_maxis_mult">,
  Intrinsic<[llvm_i64_ty], [llvm_i32_ty, llvm_i32_ty],
            [IntrNoMem, Commutative]>;
def int_maxis_multu: GCCBuiltin<"__builtin_maxis_multu">,
  Intrinsic<[llvm_i64_ty], [llvm_i32_ty, llvm_i32_ty],
            [IntrNoMem, Commutative]>;

//===----------------------------------------------------------------------===//
// Dot product with accumulate/subtract

def int_maxis_dpau_h_qbl: GCCBuiltin<"__builtin_maxis_dpau_h_qbl">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_v4i8_ty, llvm_v4i8_ty],
            [IntrNoMem]>;
def int_maxis_dpau_h_qbr: GCCBuiltin<"__builtin_maxis_dpau_h_qbr">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_v4i8_ty, llvm_v4i8_ty],
            [IntrNoMem]>;
def int_maxis_dpsu_h_qbl: GCCBuiltin<"__builtin_maxis_dpsu_h_qbl">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_v4i8_ty, llvm_v4i8_ty],
            [IntrNoMem]>;
def int_maxis_dpsu_h_qbr: GCCBuiltin<"__builtin_maxis_dpsu_h_qbr">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_v4i8_ty, llvm_v4i8_ty],
            [IntrNoMem]>;
def int_maxis_dpaq_s_w_ph: GCCBuiltin<"__builtin_maxis_dpaq_s_w_ph">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, maxis_v2q15_ty, maxis_v2q15_ty], []>;
def int_maxis_dpsq_s_w_ph: GCCBuiltin<"__builtin_maxis_dpsq_s_w_ph">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, maxis_v2q15_ty, maxis_v2q15_ty], []>;
def int_maxis_dpaq_sa_l_w: GCCBuiltin<"__builtin_maxis_dpaq_sa_l_w">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, maxis_q31_ty, maxis_q31_ty], []>;
def int_maxis_dpsq_sa_l_w: GCCBuiltin<"__builtin_maxis_dpsq_sa_l_w">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, maxis_q31_ty, maxis_q31_ty], []>;

//===----------------------------------------------------------------------===//
// Comparison

def int_maxis_cmpu_eq_qb: GCCBuiltin<"__builtin_maxis_cmpu_eq_qb">,
  Intrinsic<[], [llvm_v4i8_ty, llvm_v4i8_ty], [Commutative]>;
def int_maxis_cmpu_lt_qb: GCCBuiltin<"__builtin_maxis_cmpu_lt_qb">,
  Intrinsic<[], [llvm_v4i8_ty, llvm_v4i8_ty], []>;
def int_maxis_cmpu_le_qb: GCCBuiltin<"__builtin_maxis_cmpu_le_qb">,
  Intrinsic<[], [llvm_v4i8_ty, llvm_v4i8_ty], []>;
def int_maxis_cmpgu_eq_qb: GCCBuiltin<"__builtin_maxis_cmpgu_eq_qb">,
  Intrinsic<[llvm_i32_ty], [llvm_v4i8_ty, llvm_v4i8_ty], [Commutative]>;
def int_maxis_cmpgu_lt_qb: GCCBuiltin<"__builtin_maxis_cmpgu_lt_qb">,
  Intrinsic<[llvm_i32_ty], [llvm_v4i8_ty, llvm_v4i8_ty], []>;
def int_maxis_cmpgu_le_qb: GCCBuiltin<"__builtin_maxis_cmpgu_le_qb">,
  Intrinsic<[llvm_i32_ty], [llvm_v4i8_ty, llvm_v4i8_ty], []>;
def int_maxis_cmp_eq_ph: GCCBuiltin<"__builtin_maxis_cmp_eq_ph">,
  Intrinsic<[], [maxis_v2q15_ty, maxis_v2q15_ty], [Commutative]>;
def int_maxis_cmp_lt_ph: GCCBuiltin<"__builtin_maxis_cmp_lt_ph">,
  Intrinsic<[], [maxis_v2q15_ty, maxis_v2q15_ty], []>;
def int_maxis_cmp_le_ph: GCCBuiltin<"__builtin_maxis_cmp_le_ph">,
  Intrinsic<[], [maxis_v2q15_ty, maxis_v2q15_ty], []>;

//===----------------------------------------------------------------------===//
// Extracting

def int_maxis_extr_s_h: GCCBuiltin<"__builtin_maxis_extr_s_h">,
  Intrinsic<[llvm_i32_ty], [llvm_i64_ty, llvm_i32_ty], []>;
def int_maxis_extr_w: GCCBuiltin<"__builtin_maxis_extr_w">,
  Intrinsic<[llvm_i32_ty], [llvm_i64_ty, llvm_i32_ty], []>;
def int_maxis_extr_rs_w: GCCBuiltin<"__builtin_maxis_extr_rs_w">,
  Intrinsic<[llvm_i32_ty], [llvm_i64_ty, llvm_i32_ty], []>;
def int_maxis_extr_r_w: GCCBuiltin<"__builtin_maxis_extr_r_w">,
  Intrinsic<[llvm_i32_ty], [llvm_i64_ty, llvm_i32_ty], []>;
def int_maxis_extp: GCCBuiltin<"__builtin_maxis_extp">,
  Intrinsic<[llvm_i32_ty], [llvm_i64_ty, llvm_i32_ty], []>;
def int_maxis_extpdp: GCCBuiltin<"__builtin_maxis_extpdp">,
  Intrinsic<[llvm_i32_ty], [llvm_i64_ty, llvm_i32_ty], []>;

//===----------------------------------------------------------------------===//
// Misc

def int_maxis_wrdsp: GCCBuiltin<"__builtin_maxis_wrdsp">,
  Intrinsic<[], [llvm_i32_ty, llvm_i32_ty], []>;
def int_maxis_rddsp: GCCBuiltin<"__builtin_maxis_rddsp">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty], [IntrReadMem]>;

def int_maxis_insv: GCCBuiltin<"__builtin_maxis_insv">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty, llvm_i32_ty], [IntrReadMem]>;
def int_maxis_bitrev: GCCBuiltin<"__builtin_maxis_bitrev">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty], [IntrNoMem]>;

def int_maxis_packrl_ph: GCCBuiltin<"__builtin_maxis_packrl_ph">,
  Intrinsic<[maxis_v2q15_ty], [maxis_v2q15_ty, maxis_v2q15_ty], [IntrNoMem]>;

def int_maxis_repl_qb: GCCBuiltin<"__builtin_maxis_repl_qb">,
  Intrinsic<[llvm_v4i8_ty], [llvm_i32_ty], [IntrNoMem]>;
def int_maxis_repl_ph: GCCBuiltin<"__builtin_maxis_repl_ph">,
  Intrinsic<[maxis_v2q15_ty], [llvm_i32_ty], [IntrNoMem]>;

def int_maxis_pick_qb: GCCBuiltin<"__builtin_maxis_pick_qb">,
  Intrinsic<[llvm_v4i8_ty], [llvm_v4i8_ty, llvm_v4i8_ty], [IntrReadMem]>;
def int_maxis_pick_ph: GCCBuiltin<"__builtin_maxis_pick_ph">,
  Intrinsic<[maxis_v2q15_ty], [maxis_v2q15_ty, maxis_v2q15_ty], [IntrReadMem]>;

def int_maxis_mthlip: GCCBuiltin<"__builtin_maxis_mthlip">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_i32_ty], []>;

def int_maxis_bposge32: GCCBuiltin<"__builtin_maxis_bposge32">,
  Intrinsic<[llvm_i32_ty], [], [IntrReadMem]>;

def int_maxis_lbux: GCCBuiltin<"__builtin_maxis_lbux">,
  Intrinsic<[llvm_i32_ty], [llvm_ptr_ty, llvm_i32_ty], [IntrReadMem, IntrArgMemOnly]>;
def int_maxis_lhx: GCCBuiltin<"__builtin_maxis_lhx">,
  Intrinsic<[llvm_i32_ty], [llvm_ptr_ty, llvm_i32_ty], [IntrReadMem, IntrArgMemOnly]>;
def int_maxis_lwx: GCCBuiltin<"__builtin_maxis_lwx">,
  Intrinsic<[llvm_i32_ty], [llvm_ptr_ty, llvm_i32_ty], [IntrReadMem, IntrArgMemOnly]>;

//===----------------------------------------------------------------------===//
// MAXIS DSP Rev 2

def int_maxis_absq_s_qb: GCCBuiltin<"__builtin_maxis_absq_s_qb">,
  Intrinsic<[maxis_v4q7_ty], [maxis_v4q7_ty], []>;

def int_maxis_addqh_ph: GCCBuiltin<"__builtin_maxis_addqh_ph">,
  Intrinsic<[maxis_v2q15_ty], [maxis_v2q15_ty, maxis_v2q15_ty],
            [IntrNoMem, Commutative]>;
def int_maxis_addqh_r_ph: GCCBuiltin<"__builtin_maxis_addqh_r_ph">,
  Intrinsic<[maxis_v2q15_ty], [maxis_v2q15_ty, maxis_v2q15_ty],
            [IntrNoMem, Commutative]>;
def int_maxis_addqh_w: GCCBuiltin<"__builtin_maxis_addqh_w">,
  Intrinsic<[maxis_q31_ty], [maxis_q31_ty, maxis_q31_ty],
            [IntrNoMem, Commutative]>;
def int_maxis_addqh_r_w: GCCBuiltin<"__builtin_maxis_addqh_r_w">,
  Intrinsic<[maxis_q31_ty], [maxis_q31_ty, maxis_q31_ty],
            [IntrNoMem, Commutative]>;

def int_maxis_addu_ph: GCCBuiltin<"__builtin_maxis_addu_ph">,
  Intrinsic<[llvm_v2i16_ty], [llvm_v2i16_ty, llvm_v2i16_ty], [Commutative]>;
def int_maxis_addu_s_ph: GCCBuiltin<"__builtin_maxis_addu_s_ph">,
  Intrinsic<[llvm_v2i16_ty], [llvm_v2i16_ty, llvm_v2i16_ty], [Commutative]>;

def int_maxis_adduh_qb: GCCBuiltin<"__builtin_maxis_adduh_qb">,
  Intrinsic<[llvm_v4i8_ty], [llvm_v4i8_ty, llvm_v4i8_ty],
            [IntrNoMem, Commutative]>;
def int_maxis_adduh_r_qb: GCCBuiltin<"__builtin_maxis_adduh_r_qb">,
  Intrinsic<[llvm_v4i8_ty], [llvm_v4i8_ty, llvm_v4i8_ty],
            [IntrNoMem, Commutative]>;

def int_maxis_append: GCCBuiltin<"__builtin_maxis_append">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty, llvm_i32_ty, llvm_i32_ty],
  [IntrNoMem]>;
def int_maxis_balign: GCCBuiltin<"__builtin_maxis_balign">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty, llvm_i32_ty, llvm_i32_ty],
  [IntrNoMem]>;

def int_maxis_cmpgdu_eq_qb: GCCBuiltin<"__builtin_maxis_cmpgdu_eq_qb">,
  Intrinsic<[llvm_i32_ty], [llvm_v4i8_ty, llvm_v4i8_ty], [Commutative]>;
def int_maxis_cmpgdu_lt_qb: GCCBuiltin<"__builtin_maxis_cmpgdu_lt_qb">,
  Intrinsic<[llvm_i32_ty], [llvm_v4i8_ty, llvm_v4i8_ty], []>;
def int_maxis_cmpgdu_le_qb: GCCBuiltin<"__builtin_maxis_cmpgdu_le_qb">,
  Intrinsic<[llvm_i32_ty], [llvm_v4i8_ty, llvm_v4i8_ty], []>;

def int_maxis_dpa_w_ph: GCCBuiltin<"__builtin_maxis_dpa_w_ph">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_v2i16_ty, llvm_v2i16_ty],
            [IntrNoMem]>;
def int_maxis_dps_w_ph: GCCBuiltin<"__builtin_maxis_dps_w_ph">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_v2i16_ty, llvm_v2i16_ty],
            [IntrNoMem]>;

def int_maxis_dpaqx_s_w_ph: GCCBuiltin<"__builtin_maxis_dpaqx_s_w_ph">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, maxis_v2q15_ty, maxis_v2q15_ty], []>;
def int_maxis_dpaqx_sa_w_ph: GCCBuiltin<"__builtin_maxis_dpaqx_sa_w_ph">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, maxis_v2q15_ty, maxis_v2q15_ty], []>;
def int_maxis_dpax_w_ph: GCCBuiltin<"__builtin_maxis_dpax_w_ph">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_v2i16_ty, llvm_v2i16_ty],
            [IntrNoMem]>;
def int_maxis_dpsx_w_ph: GCCBuiltin<"__builtin_maxis_dpsx_w_ph">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_v2i16_ty, llvm_v2i16_ty],
            [IntrNoMem]>;
def int_maxis_dpsqx_s_w_ph: GCCBuiltin<"__builtin_maxis_dpsqx_s_w_ph">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, maxis_v2q15_ty, maxis_v2q15_ty], []>;
def int_maxis_dpsqx_sa_w_ph: GCCBuiltin<"__builtin_maxis_dpsqx_sa_w_ph">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, maxis_v2q15_ty, maxis_v2q15_ty], []>;

def int_maxis_mul_ph: GCCBuiltin<"__builtin_maxis_mul_ph">,
  Intrinsic<[llvm_v2i16_ty], [llvm_v2i16_ty, llvm_v2i16_ty], [Commutative]>;
def int_maxis_mul_s_ph: GCCBuiltin<"__builtin_maxis_mul_s_ph">,
  Intrinsic<[llvm_v2i16_ty], [llvm_v2i16_ty, llvm_v2i16_ty], [Commutative]>;

def int_maxis_mulq_rs_w: GCCBuiltin<"__builtin_maxis_mulq_rs_w">,
  Intrinsic<[maxis_q31_ty], [maxis_q31_ty, maxis_q31_ty], [Commutative]>;
def int_maxis_mulq_s_ph: GCCBuiltin<"__builtin_maxis_mulq_s_ph">,
  Intrinsic<[maxis_v2q15_ty], [maxis_v2q15_ty, maxis_v2q15_ty], [Commutative]>;
def int_maxis_mulq_s_w: GCCBuiltin<"__builtin_maxis_mulq_s_w">,
  Intrinsic<[maxis_q31_ty], [maxis_q31_ty, maxis_q31_ty], [Commutative]>;
def int_maxis_mulsa_w_ph: GCCBuiltin<"__builtin_maxis_mulsa_w_ph">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_v2i16_ty, llvm_v2i16_ty],
            [IntrNoMem]>;

def int_maxis_precr_qb_ph: GCCBuiltin<"__builtin_maxis_precr_qb_ph">,
  Intrinsic<[llvm_v4i8_ty], [llvm_v2i16_ty, llvm_v2i16_ty], []>;
def int_maxis_precr_sra_ph_w: GCCBuiltin<"__builtin_maxis_precr_sra_ph_w">,
  Intrinsic<[llvm_v2i16_ty], [llvm_i32_ty, llvm_i32_ty, llvm_i32_ty],
            [IntrNoMem]>;
def int_maxis_precr_sra_r_ph_w: GCCBuiltin<"__builtin_maxis_precr_sra_r_ph_w">,
  Intrinsic<[llvm_v2i16_ty], [llvm_i32_ty, llvm_i32_ty, llvm_i32_ty],
            [IntrNoMem]>;

def int_maxis_prepend: GCCBuiltin<"__builtin_maxis_prepend">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty, llvm_i32_ty, llvm_i32_ty],
  [IntrNoMem]>;

def int_maxis_shra_qb: GCCBuiltin<"__builtin_maxis_shra_qb">,
  Intrinsic<[llvm_v4i8_ty], [llvm_v4i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_shra_r_qb: GCCBuiltin<"__builtin_maxis_shra_r_qb">,
  Intrinsic<[llvm_v4i8_ty], [llvm_v4i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_shrl_ph: GCCBuiltin<"__builtin_maxis_shrl_ph">,
  Intrinsic<[llvm_v2i16_ty], [llvm_v2i16_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_subqh_ph: GCCBuiltin<"__builtin_maxis_subqh_ph">,
  Intrinsic<[maxis_v2q15_ty], [maxis_v2q15_ty, maxis_v2q15_ty], [IntrNoMem]>;
def int_maxis_subqh_r_ph: GCCBuiltin<"__builtin_maxis_subqh_r_ph">,
  Intrinsic<[maxis_v2q15_ty], [maxis_v2q15_ty, maxis_v2q15_ty], [IntrNoMem]>;
def int_maxis_subqh_w: GCCBuiltin<"__builtin_maxis_subqh_w">,
  Intrinsic<[maxis_q31_ty], [maxis_q31_ty, maxis_q31_ty], [IntrNoMem]>;
def int_maxis_subqh_r_w: GCCBuiltin<"__builtin_maxis_subqh_r_w">,
  Intrinsic<[maxis_q31_ty], [maxis_q31_ty, maxis_q31_ty], [IntrNoMem]>;

def int_maxis_subu_ph: GCCBuiltin<"__builtin_maxis_subu_ph">,
  Intrinsic<[llvm_v2i16_ty], [llvm_v2i16_ty, llvm_v2i16_ty], []>;
def int_maxis_subu_s_ph: GCCBuiltin<"__builtin_maxis_subu_s_ph">,
  Intrinsic<[llvm_v2i16_ty], [llvm_v2i16_ty, llvm_v2i16_ty], []>;

def int_maxis_subuh_qb: GCCBuiltin<"__builtin_maxis_subuh_qb">,
  Intrinsic<[llvm_v4i8_ty], [llvm_v4i8_ty, llvm_v4i8_ty], [IntrNoMem]>;
def int_maxis_subuh_r_qb: GCCBuiltin<"__builtin_maxis_subuh_r_qb">,
  Intrinsic<[llvm_v4i8_ty], [llvm_v4i8_ty, llvm_v4i8_ty], [IntrNoMem]>;

//===----------------------------------------------------------------------===//
// MAXIS MSA

//===----------------------------------------------------------------------===//
// Addition/subtraction

def int_maxis_add_a_b : GCCBuiltin<"__builtin_msa_add_a_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_add_a_h : GCCBuiltin<"__builtin_msa_add_a_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_add_a_w : GCCBuiltin<"__builtin_msa_add_a_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_add_a_d : GCCBuiltin<"__builtin_msa_add_a_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty],
  [Commutative, IntrNoMem]>;

def int_maxis_adds_a_b : GCCBuiltin<"__builtin_msa_adds_a_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_adds_a_h : GCCBuiltin<"__builtin_msa_adds_a_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_adds_a_w : GCCBuiltin<"__builtin_msa_adds_a_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_adds_a_d : GCCBuiltin<"__builtin_msa_adds_a_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty],
  [Commutative, IntrNoMem]>;

def int_maxis_adds_s_b : GCCBuiltin<"__builtin_msa_adds_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_adds_s_h : GCCBuiltin<"__builtin_msa_adds_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_adds_s_w : GCCBuiltin<"__builtin_msa_adds_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_adds_s_d : GCCBuiltin<"__builtin_msa_adds_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty],
  [Commutative, IntrNoMem]>;

def int_maxis_adds_u_b : GCCBuiltin<"__builtin_msa_adds_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_adds_u_h : GCCBuiltin<"__builtin_msa_adds_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_adds_u_w : GCCBuiltin<"__builtin_msa_adds_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_adds_u_d : GCCBuiltin<"__builtin_msa_adds_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty],
  [Commutative, IntrNoMem]>;

def int_maxis_addv_b : GCCBuiltin<"__builtin_msa_addv_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_addv_h : GCCBuiltin<"__builtin_msa_addv_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_addv_w : GCCBuiltin<"__builtin_msa_addv_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_addv_d : GCCBuiltin<"__builtin_msa_addv_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty],
  [Commutative, IntrNoMem]>;

def int_maxis_addvi_b : GCCBuiltin<"__builtin_msa_addvi_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_addvi_h : GCCBuiltin<"__builtin_msa_addvi_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_addvi_w : GCCBuiltin<"__builtin_msa_addvi_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_addvi_d : GCCBuiltin<"__builtin_msa_addvi_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty],
  [Commutative, IntrNoMem]>;

def int_maxis_and_v : GCCBuiltin<"__builtin_msa_and_v">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;

def int_maxis_andi_b : GCCBuiltin<"__builtin_msa_andi_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_asub_s_b : GCCBuiltin<"__builtin_msa_asub_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_asub_s_h : GCCBuiltin<"__builtin_msa_asub_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_asub_s_w : GCCBuiltin<"__builtin_msa_asub_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_asub_s_d : GCCBuiltin<"__builtin_msa_asub_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_asub_u_b : GCCBuiltin<"__builtin_msa_asub_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_asub_u_h : GCCBuiltin<"__builtin_msa_asub_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_asub_u_w : GCCBuiltin<"__builtin_msa_asub_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_asub_u_d : GCCBuiltin<"__builtin_msa_asub_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_ave_s_b : GCCBuiltin<"__builtin_msa_ave_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_ave_s_h : GCCBuiltin<"__builtin_msa_ave_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_ave_s_w : GCCBuiltin<"__builtin_msa_ave_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_ave_s_d : GCCBuiltin<"__builtin_msa_ave_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty],
  [Commutative, IntrNoMem]>;

def int_maxis_ave_u_b : GCCBuiltin<"__builtin_msa_ave_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_ave_u_h : GCCBuiltin<"__builtin_msa_ave_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_ave_u_w : GCCBuiltin<"__builtin_msa_ave_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_ave_u_d : GCCBuiltin<"__builtin_msa_ave_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty],
  [Commutative, IntrNoMem]>;

def int_maxis_aver_s_b : GCCBuiltin<"__builtin_msa_aver_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_aver_s_h : GCCBuiltin<"__builtin_msa_aver_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_aver_s_w : GCCBuiltin<"__builtin_msa_aver_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_aver_s_d : GCCBuiltin<"__builtin_msa_aver_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty],
  [Commutative, IntrNoMem]>;

def int_maxis_aver_u_b : GCCBuiltin<"__builtin_msa_aver_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_aver_u_h : GCCBuiltin<"__builtin_msa_aver_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_aver_u_w : GCCBuiltin<"__builtin_msa_aver_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty],
  [Commutative, IntrNoMem]>;
def int_maxis_aver_u_d : GCCBuiltin<"__builtin_msa_aver_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty],
  [Commutative, IntrNoMem]>;

def int_maxis_bclr_b : GCCBuiltin<"__builtin_msa_bclr_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_bclr_h : GCCBuiltin<"__builtin_msa_bclr_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_bclr_w : GCCBuiltin<"__builtin_msa_bclr_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_bclr_d : GCCBuiltin<"__builtin_msa_bclr_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_bclri_b : GCCBuiltin<"__builtin_msa_bclri_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_bclri_h : GCCBuiltin<"__builtin_msa_bclri_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_bclri_w : GCCBuiltin<"__builtin_msa_bclri_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_bclri_d : GCCBuiltin<"__builtin_msa_bclri_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_binsl_b : GCCBuiltin<"__builtin_msa_binsl_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_v16i8_ty],
            [IntrNoMem]>;
def int_maxis_binsl_h : GCCBuiltin<"__builtin_msa_binsl_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty, llvm_v8i16_ty],
            [IntrNoMem]>;
def int_maxis_binsl_w : GCCBuiltin<"__builtin_msa_binsl_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty, llvm_v4i32_ty],
            [IntrNoMem]>;
def int_maxis_binsl_d : GCCBuiltin<"__builtin_msa_binsl_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty, llvm_v2i64_ty],
            [IntrNoMem]>;

def int_maxis_binsli_b : GCCBuiltin<"__builtin_msa_binsli_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_i32_ty],
            [IntrNoMem]>;
def int_maxis_binsli_h : GCCBuiltin<"__builtin_msa_binsli_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty, llvm_i32_ty],
            [IntrNoMem]>;
def int_maxis_binsli_w : GCCBuiltin<"__builtin_msa_binsli_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty, llvm_i32_ty],
            [IntrNoMem]>;
def int_maxis_binsli_d : GCCBuiltin<"__builtin_msa_binsli_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty, llvm_i32_ty],
            [IntrNoMem]>;

def int_maxis_binsr_b : GCCBuiltin<"__builtin_msa_binsr_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_v16i8_ty],
            [IntrNoMem]>;
def int_maxis_binsr_h : GCCBuiltin<"__builtin_msa_binsr_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty, llvm_v8i16_ty],
            [IntrNoMem]>;
def int_maxis_binsr_w : GCCBuiltin<"__builtin_msa_binsr_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty, llvm_v4i32_ty],
            [IntrNoMem]>;
def int_maxis_binsr_d : GCCBuiltin<"__builtin_msa_binsr_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty, llvm_v2i64_ty],
            [IntrNoMem]>;

def int_maxis_binsri_b : GCCBuiltin<"__builtin_msa_binsri_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_i32_ty],
            [IntrNoMem]>;
def int_maxis_binsri_h : GCCBuiltin<"__builtin_msa_binsri_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty, llvm_i32_ty],
            [IntrNoMem]>;
def int_maxis_binsri_w : GCCBuiltin<"__builtin_msa_binsri_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty, llvm_i32_ty],
            [IntrNoMem]>;
def int_maxis_binsri_d : GCCBuiltin<"__builtin_msa_binsri_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty, llvm_i32_ty],
            [IntrNoMem]>;

def int_maxis_bmnz_v : GCCBuiltin<"__builtin_msa_bmnz_v">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_v16i8_ty],
            [IntrNoMem]>;

def int_maxis_bmnzi_b : GCCBuiltin<"__builtin_msa_bmnzi_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_i32_ty],
            [IntrNoMem]>;

def int_maxis_bmz_v : GCCBuiltin<"__builtin_msa_bmz_v">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_v16i8_ty],
            [IntrNoMem]>;

def int_maxis_bmzi_b : GCCBuiltin<"__builtin_msa_bmzi_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_i32_ty],
            [IntrNoMem]>;

def int_maxis_bneg_b : GCCBuiltin<"__builtin_msa_bneg_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_bneg_h : GCCBuiltin<"__builtin_msa_bneg_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_bneg_w : GCCBuiltin<"__builtin_msa_bneg_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_bneg_d : GCCBuiltin<"__builtin_msa_bneg_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_bnegi_b : GCCBuiltin<"__builtin_msa_bnegi_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_bnegi_h : GCCBuiltin<"__builtin_msa_bnegi_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_bnegi_w : GCCBuiltin<"__builtin_msa_bnegi_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_bnegi_d : GCCBuiltin<"__builtin_msa_bnegi_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_bnz_b : GCCBuiltin<"__builtin_msa_bnz_b">,
  Intrinsic<[llvm_i32_ty], [llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_bnz_h : GCCBuiltin<"__builtin_msa_bnz_h">,
  Intrinsic<[llvm_i32_ty], [llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_bnz_w : GCCBuiltin<"__builtin_msa_bnz_w">,
  Intrinsic<[llvm_i32_ty], [llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_bnz_d : GCCBuiltin<"__builtin_msa_bnz_d">,
  Intrinsic<[llvm_i32_ty], [llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_bnz_v : GCCBuiltin<"__builtin_msa_bnz_v">,
  Intrinsic<[llvm_i32_ty], [llvm_v16i8_ty], [IntrNoMem]>;

def int_maxis_bsel_v : GCCBuiltin<"__builtin_msa_bsel_v">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_v16i8_ty],
            [IntrNoMem]>;

def int_maxis_bseli_b : GCCBuiltin<"__builtin_msa_bseli_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_i32_ty],
            [IntrNoMem]>;

def int_maxis_bset_b : GCCBuiltin<"__builtin_msa_bset_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_bset_h : GCCBuiltin<"__builtin_msa_bset_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_bset_w : GCCBuiltin<"__builtin_msa_bset_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_bset_d : GCCBuiltin<"__builtin_msa_bset_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_bseti_b : GCCBuiltin<"__builtin_msa_bseti_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_bseti_h : GCCBuiltin<"__builtin_msa_bseti_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_bseti_w : GCCBuiltin<"__builtin_msa_bseti_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_bseti_d : GCCBuiltin<"__builtin_msa_bseti_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_bz_b : GCCBuiltin<"__builtin_msa_bz_b">,
  Intrinsic<[llvm_i32_ty], [llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_bz_h : GCCBuiltin<"__builtin_msa_bz_h">,
  Intrinsic<[llvm_i32_ty], [llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_bz_w : GCCBuiltin<"__builtin_msa_bz_w">,
  Intrinsic<[llvm_i32_ty], [llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_bz_d : GCCBuiltin<"__builtin_msa_bz_d">,
  Intrinsic<[llvm_i32_ty], [llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_bz_v : GCCBuiltin<"__builtin_msa_bz_v">,
  Intrinsic<[llvm_i32_ty], [llvm_v16i8_ty], [IntrNoMem]>;

def int_maxis_ceq_b : GCCBuiltin<"__builtin_msa_ceq_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_ceq_h : GCCBuiltin<"__builtin_msa_ceq_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_ceq_w : GCCBuiltin<"__builtin_msa_ceq_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_ceq_d : GCCBuiltin<"__builtin_msa_ceq_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_ceqi_b : GCCBuiltin<"__builtin_msa_ceqi_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_ceqi_h : GCCBuiltin<"__builtin_msa_ceqi_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_ceqi_w : GCCBuiltin<"__builtin_msa_ceqi_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_ceqi_d : GCCBuiltin<"__builtin_msa_ceqi_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_cfcmsa : GCCBuiltin<"__builtin_msa_cfcmsa">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty], []>;

def int_maxis_cle_s_b : GCCBuiltin<"__builtin_msa_cle_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_cle_s_h : GCCBuiltin<"__builtin_msa_cle_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_cle_s_w : GCCBuiltin<"__builtin_msa_cle_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_cle_s_d : GCCBuiltin<"__builtin_msa_cle_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_cle_u_b : GCCBuiltin<"__builtin_msa_cle_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_cle_u_h : GCCBuiltin<"__builtin_msa_cle_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_cle_u_w : GCCBuiltin<"__builtin_msa_cle_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_cle_u_d : GCCBuiltin<"__builtin_msa_cle_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_clei_s_b : GCCBuiltin<"__builtin_msa_clei_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_clei_s_h : GCCBuiltin<"__builtin_msa_clei_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_clei_s_w : GCCBuiltin<"__builtin_msa_clei_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_clei_s_d : GCCBuiltin<"__builtin_msa_clei_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_clei_u_b : GCCBuiltin<"__builtin_msa_clei_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_clei_u_h : GCCBuiltin<"__builtin_msa_clei_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_clei_u_w : GCCBuiltin<"__builtin_msa_clei_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_clei_u_d : GCCBuiltin<"__builtin_msa_clei_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_clt_s_b : GCCBuiltin<"__builtin_msa_clt_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_clt_s_h : GCCBuiltin<"__builtin_msa_clt_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_clt_s_w : GCCBuiltin<"__builtin_msa_clt_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_clt_s_d : GCCBuiltin<"__builtin_msa_clt_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_clt_u_b : GCCBuiltin<"__builtin_msa_clt_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_clt_u_h : GCCBuiltin<"__builtin_msa_clt_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_clt_u_w : GCCBuiltin<"__builtin_msa_clt_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_clt_u_d : GCCBuiltin<"__builtin_msa_clt_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_clti_s_b : GCCBuiltin<"__builtin_msa_clti_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_clti_s_h : GCCBuiltin<"__builtin_msa_clti_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_clti_s_w : GCCBuiltin<"__builtin_msa_clti_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_clti_s_d : GCCBuiltin<"__builtin_msa_clti_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_clti_u_b : GCCBuiltin<"__builtin_msa_clti_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_clti_u_h : GCCBuiltin<"__builtin_msa_clti_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_clti_u_w : GCCBuiltin<"__builtin_msa_clti_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_clti_u_d : GCCBuiltin<"__builtin_msa_clti_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_copy_s_b : GCCBuiltin<"__builtin_msa_copy_s_b">,
  Intrinsic<[llvm_i32_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_copy_s_h : GCCBuiltin<"__builtin_msa_copy_s_h">,
  Intrinsic<[llvm_i32_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_copy_s_w : GCCBuiltin<"__builtin_msa_copy_s_w">,
  Intrinsic<[llvm_i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_copy_s_d : GCCBuiltin<"__builtin_msa_copy_s_d">,
  Intrinsic<[llvm_i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_copy_u_b : GCCBuiltin<"__builtin_msa_copy_u_b">,
  Intrinsic<[llvm_i32_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_copy_u_h : GCCBuiltin<"__builtin_msa_copy_u_h">,
  Intrinsic<[llvm_i32_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_copy_u_w : GCCBuiltin<"__builtin_msa_copy_u_w">,
  Intrinsic<[llvm_i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_copy_u_d : GCCBuiltin<"__builtin_msa_copy_u_d">,
  Intrinsic<[llvm_i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_ctcmsa : GCCBuiltin<"__builtin_msa_ctcmsa">,
  Intrinsic<[], [llvm_i32_ty, llvm_i32_ty], []>;

def int_maxis_div_s_b : GCCBuiltin<"__builtin_msa_div_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_div_s_h : GCCBuiltin<"__builtin_msa_div_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_div_s_w : GCCBuiltin<"__builtin_msa_div_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_div_s_d : GCCBuiltin<"__builtin_msa_div_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_div_u_b : GCCBuiltin<"__builtin_msa_div_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_div_u_h : GCCBuiltin<"__builtin_msa_div_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_div_u_w : GCCBuiltin<"__builtin_msa_div_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_div_u_d : GCCBuiltin<"__builtin_msa_div_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

// This instruction is part of the MSA spec but it does not share the
// __builtin_msa prefix because it operates on GP registers.
def int_maxis_dlsa : GCCBuiltin<"__builtin_maxis_dlsa">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_i64_ty, llvm_i32_ty],
            [IntrNoMem]>;

def int_maxis_dotp_s_h : GCCBuiltin<"__builtin_msa_dotp_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_dotp_s_w : GCCBuiltin<"__builtin_msa_dotp_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_dotp_s_d : GCCBuiltin<"__builtin_msa_dotp_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;

def int_maxis_dotp_u_h : GCCBuiltin<"__builtin_msa_dotp_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_dotp_u_w : GCCBuiltin<"__builtin_msa_dotp_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_dotp_u_d : GCCBuiltin<"__builtin_msa_dotp_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;

def int_maxis_dpadd_s_h : GCCBuiltin<"__builtin_msa_dpadd_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v16i8_ty, llvm_v16i8_ty],
  [IntrNoMem]>;
def int_maxis_dpadd_s_w : GCCBuiltin<"__builtin_msa_dpadd_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v8i16_ty, llvm_v8i16_ty],
  [IntrNoMem]>;
def int_maxis_dpadd_s_d : GCCBuiltin<"__builtin_msa_dpadd_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v4i32_ty, llvm_v4i32_ty],
  [IntrNoMem]>;

def int_maxis_dpadd_u_h : GCCBuiltin<"__builtin_msa_dpadd_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v16i8_ty, llvm_v16i8_ty],
  [IntrNoMem]>;
def int_maxis_dpadd_u_w : GCCBuiltin<"__builtin_msa_dpadd_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v8i16_ty, llvm_v8i16_ty],
  [IntrNoMem]>;
def int_maxis_dpadd_u_d : GCCBuiltin<"__builtin_msa_dpadd_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v4i32_ty, llvm_v4i32_ty],
  [IntrNoMem]>;

def int_maxis_dpsub_s_h : GCCBuiltin<"__builtin_msa_dpsub_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v16i8_ty, llvm_v16i8_ty],
  [IntrNoMem]>;
def int_maxis_dpsub_s_w : GCCBuiltin<"__builtin_msa_dpsub_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v8i16_ty, llvm_v8i16_ty],
  [IntrNoMem]>;
def int_maxis_dpsub_s_d : GCCBuiltin<"__builtin_msa_dpsub_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v4i32_ty, llvm_v4i32_ty],
  [IntrNoMem]>;

def int_maxis_dpsub_u_h : GCCBuiltin<"__builtin_msa_dpsub_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v16i8_ty, llvm_v16i8_ty],
  [IntrNoMem]>;
def int_maxis_dpsub_u_w : GCCBuiltin<"__builtin_msa_dpsub_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v8i16_ty, llvm_v8i16_ty],
  [IntrNoMem]>;
def int_maxis_dpsub_u_d : GCCBuiltin<"__builtin_msa_dpsub_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v4i32_ty, llvm_v4i32_ty],
  [IntrNoMem]>;

def int_maxis_fadd_w : GCCBuiltin<"__builtin_msa_fadd_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fadd_d : GCCBuiltin<"__builtin_msa_fadd_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fcaf_w : GCCBuiltin<"__builtin_msa_fcaf_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fcaf_d : GCCBuiltin<"__builtin_msa_fcaf_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fceq_w : GCCBuiltin<"__builtin_msa_fceq_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fceq_d : GCCBuiltin<"__builtin_msa_fceq_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fcle_w : GCCBuiltin<"__builtin_msa_fcle_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fcle_d : GCCBuiltin<"__builtin_msa_fcle_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fclt_w : GCCBuiltin<"__builtin_msa_fclt_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fclt_d : GCCBuiltin<"__builtin_msa_fclt_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fclass_w : GCCBuiltin<"__builtin_msa_fclass_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fclass_d : GCCBuiltin<"__builtin_msa_fclass_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fcne_w : GCCBuiltin<"__builtin_msa_fcne_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fcne_d : GCCBuiltin<"__builtin_msa_fcne_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fcor_w : GCCBuiltin<"__builtin_msa_fcor_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fcor_d : GCCBuiltin<"__builtin_msa_fcor_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fcueq_w : GCCBuiltin<"__builtin_msa_fcueq_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fcueq_d : GCCBuiltin<"__builtin_msa_fcueq_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fcule_w : GCCBuiltin<"__builtin_msa_fcule_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fcule_d : GCCBuiltin<"__builtin_msa_fcule_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fcult_w : GCCBuiltin<"__builtin_msa_fcult_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fcult_d : GCCBuiltin<"__builtin_msa_fcult_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fcun_w : GCCBuiltin<"__builtin_msa_fcun_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fcun_d : GCCBuiltin<"__builtin_msa_fcun_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fcune_w : GCCBuiltin<"__builtin_msa_fcune_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fcune_d : GCCBuiltin<"__builtin_msa_fcune_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fdiv_w : GCCBuiltin<"__builtin_msa_fdiv_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fdiv_d : GCCBuiltin<"__builtin_msa_fdiv_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fexdo_h : GCCBuiltin<"__builtin_msa_fexdo_h">,
  Intrinsic<[llvm_v8f16_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fexdo_w : GCCBuiltin<"__builtin_msa_fexdo_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fexp2_w : GCCBuiltin<"__builtin_msa_fexp2_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_fexp2_d : GCCBuiltin<"__builtin_msa_fexp2_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_fexupl_w : GCCBuiltin<"__builtin_msa_fexupl_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v8f16_ty], [IntrNoMem]>;
def int_maxis_fexupl_d : GCCBuiltin<"__builtin_msa_fexupl_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v4f32_ty], [IntrNoMem]>;

def int_maxis_fexupr_w : GCCBuiltin<"__builtin_msa_fexupr_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v8f16_ty], [IntrNoMem]>;
def int_maxis_fexupr_d : GCCBuiltin<"__builtin_msa_fexupr_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v4f32_ty], [IntrNoMem]>;

def int_maxis_ffint_s_w : GCCBuiltin<"__builtin_msa_ffint_s_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_ffint_s_d : GCCBuiltin<"__builtin_msa_ffint_s_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_ffint_u_w : GCCBuiltin<"__builtin_msa_ffint_u_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_ffint_u_d : GCCBuiltin<"__builtin_msa_ffint_u_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_ffql_w : GCCBuiltin<"__builtin_msa_ffql_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_ffql_d : GCCBuiltin<"__builtin_msa_ffql_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v4i32_ty], [IntrNoMem]>;

def int_maxis_ffqr_w : GCCBuiltin<"__builtin_msa_ffqr_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_ffqr_d : GCCBuiltin<"__builtin_msa_ffqr_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v4i32_ty], [IntrNoMem]>;

def int_maxis_fill_b : GCCBuiltin<"__builtin_msa_fill_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_i32_ty], [IntrNoMem]>;
def int_maxis_fill_h : GCCBuiltin<"__builtin_msa_fill_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_i32_ty], [IntrNoMem]>;
def int_maxis_fill_w : GCCBuiltin<"__builtin_msa_fill_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_i32_ty], [IntrNoMem]>;
def int_maxis_fill_d : GCCBuiltin<"__builtin_msa_fill_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_i64_ty], [IntrNoMem]>;

def int_maxis_flog2_w : GCCBuiltin<"__builtin_msa_flog2_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_flog2_d : GCCBuiltin<"__builtin_msa_flog2_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fmadd_w : GCCBuiltin<"__builtin_msa_fmadd_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty, llvm_v4f32_ty, llvm_v4f32_ty],
            [IntrNoMem]>;
def int_maxis_fmadd_d : GCCBuiltin<"__builtin_msa_fmadd_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty, llvm_v2f64_ty, llvm_v2f64_ty],
            [IntrNoMem]>;

def int_maxis_fmax_w : GCCBuiltin<"__builtin_msa_fmax_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fmax_d : GCCBuiltin<"__builtin_msa_fmax_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fmax_a_w : GCCBuiltin<"__builtin_msa_fmax_a_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fmax_a_d : GCCBuiltin<"__builtin_msa_fmax_a_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fmin_w : GCCBuiltin<"__builtin_msa_fmin_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fmin_d : GCCBuiltin<"__builtin_msa_fmin_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fmin_a_w : GCCBuiltin<"__builtin_msa_fmin_a_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fmin_a_d : GCCBuiltin<"__builtin_msa_fmin_a_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fmsub_w : GCCBuiltin<"__builtin_msa_fmsub_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty, llvm_v4f32_ty, llvm_v4f32_ty],
            [IntrNoMem]>;
def int_maxis_fmsub_d : GCCBuiltin<"__builtin_msa_fmsub_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty, llvm_v2f64_ty, llvm_v2f64_ty],
            [IntrNoMem]>;

def int_maxis_fmul_w : GCCBuiltin<"__builtin_msa_fmul_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fmul_d : GCCBuiltin<"__builtin_msa_fmul_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_frint_w : GCCBuiltin<"__builtin_msa_frint_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_frint_d : GCCBuiltin<"__builtin_msa_frint_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_frcp_w : GCCBuiltin<"__builtin_msa_frcp_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_frcp_d : GCCBuiltin<"__builtin_msa_frcp_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_frsqrt_w : GCCBuiltin<"__builtin_msa_frsqrt_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_frsqrt_d : GCCBuiltin<"__builtin_msa_frsqrt_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fsaf_w : GCCBuiltin<"__builtin_msa_fsaf_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fsaf_d : GCCBuiltin<"__builtin_msa_fsaf_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fseq_w : GCCBuiltin<"__builtin_msa_fseq_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fseq_d : GCCBuiltin<"__builtin_msa_fseq_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fsle_w : GCCBuiltin<"__builtin_msa_fsle_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fsle_d : GCCBuiltin<"__builtin_msa_fsle_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fslt_w : GCCBuiltin<"__builtin_msa_fslt_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fslt_d : GCCBuiltin<"__builtin_msa_fslt_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fsne_w : GCCBuiltin<"__builtin_msa_fsne_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fsne_d : GCCBuiltin<"__builtin_msa_fsne_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fsor_w : GCCBuiltin<"__builtin_msa_fsor_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fsor_d : GCCBuiltin<"__builtin_msa_fsor_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fsqrt_w : GCCBuiltin<"__builtin_msa_fsqrt_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fsqrt_d : GCCBuiltin<"__builtin_msa_fsqrt_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fsub_w : GCCBuiltin<"__builtin_msa_fsub_w">,
  Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fsub_d : GCCBuiltin<"__builtin_msa_fsub_d">,
  Intrinsic<[llvm_v2f64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fsueq_w : GCCBuiltin<"__builtin_msa_fsueq_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fsueq_d : GCCBuiltin<"__builtin_msa_fsueq_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fsule_w : GCCBuiltin<"__builtin_msa_fsule_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fsule_d : GCCBuiltin<"__builtin_msa_fsule_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fsult_w : GCCBuiltin<"__builtin_msa_fsult_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fsult_d : GCCBuiltin<"__builtin_msa_fsult_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fsun_w : GCCBuiltin<"__builtin_msa_fsun_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fsun_d : GCCBuiltin<"__builtin_msa_fsun_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_fsune_w : GCCBuiltin<"__builtin_msa_fsune_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_fsune_d : GCCBuiltin<"__builtin_msa_fsune_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_ftint_s_w : GCCBuiltin<"__builtin_msa_ftint_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_ftint_s_d : GCCBuiltin<"__builtin_msa_ftint_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_ftint_u_w : GCCBuiltin<"__builtin_msa_ftint_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_ftint_u_d : GCCBuiltin<"__builtin_msa_ftint_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_ftq_h : GCCBuiltin<"__builtin_msa_ftq_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_ftq_w : GCCBuiltin<"__builtin_msa_ftq_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v2f64_ty, llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_ftrunc_s_w : GCCBuiltin<"__builtin_msa_ftrunc_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_ftrunc_s_d : GCCBuiltin<"__builtin_msa_ftrunc_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_ftrunc_u_w : GCCBuiltin<"__builtin_msa_ftrunc_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty], [IntrNoMem]>;
def int_maxis_ftrunc_u_d : GCCBuiltin<"__builtin_msa_ftrunc_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2f64_ty], [IntrNoMem]>;

def int_maxis_hadd_s_h : GCCBuiltin<"__builtin_msa_hadd_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_hadd_s_w : GCCBuiltin<"__builtin_msa_hadd_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_hadd_s_d : GCCBuiltin<"__builtin_msa_hadd_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;

def int_maxis_hadd_u_h : GCCBuiltin<"__builtin_msa_hadd_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_hadd_u_w : GCCBuiltin<"__builtin_msa_hadd_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_hadd_u_d : GCCBuiltin<"__builtin_msa_hadd_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;

def int_maxis_hsub_s_h : GCCBuiltin<"__builtin_msa_hsub_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_hsub_s_w : GCCBuiltin<"__builtin_msa_hsub_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_hsub_s_d : GCCBuiltin<"__builtin_msa_hsub_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;

def int_maxis_hsub_u_h : GCCBuiltin<"__builtin_msa_hsub_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_hsub_u_w : GCCBuiltin<"__builtin_msa_hsub_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_hsub_u_d : GCCBuiltin<"__builtin_msa_hsub_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;

def int_maxis_ilvev_b : GCCBuiltin<"__builtin_msa_ilvev_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_ilvev_h : GCCBuiltin<"__builtin_msa_ilvev_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_ilvev_w : GCCBuiltin<"__builtin_msa_ilvev_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_ilvev_d : GCCBuiltin<"__builtin_msa_ilvev_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_ilvl_b : GCCBuiltin<"__builtin_msa_ilvl_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_ilvl_h : GCCBuiltin<"__builtin_msa_ilvl_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_ilvl_w : GCCBuiltin<"__builtin_msa_ilvl_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_ilvl_d : GCCBuiltin<"__builtin_msa_ilvl_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_ilvod_b : GCCBuiltin<"__builtin_msa_ilvod_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_ilvod_h : GCCBuiltin<"__builtin_msa_ilvod_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_ilvod_w : GCCBuiltin<"__builtin_msa_ilvod_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_ilvod_d : GCCBuiltin<"__builtin_msa_ilvod_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_ilvr_b : GCCBuiltin<"__builtin_msa_ilvr_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_ilvr_h : GCCBuiltin<"__builtin_msa_ilvr_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_ilvr_w : GCCBuiltin<"__builtin_msa_ilvr_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_ilvr_d : GCCBuiltin<"__builtin_msa_ilvr_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_insert_b : GCCBuiltin<"__builtin_msa_insert_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty, llvm_i32_ty],
  [IntrNoMem]>;
def int_maxis_insert_h : GCCBuiltin<"__builtin_msa_insert_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty, llvm_i32_ty],
  [IntrNoMem]>;
def int_maxis_insert_w : GCCBuiltin<"__builtin_msa_insert_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty, llvm_i32_ty],
  [IntrNoMem]>;
def int_maxis_insert_d : GCCBuiltin<"__builtin_msa_insert_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty, llvm_i64_ty],
  [IntrNoMem]>;

def int_maxis_insve_b : GCCBuiltin<"__builtin_msa_insve_b">,
  Intrinsic<[llvm_v16i8_ty],
            [llvm_v16i8_ty, llvm_i32_ty, llvm_v16i8_ty],
            [IntrNoMem]>;
def int_maxis_insve_h : GCCBuiltin<"__builtin_msa_insve_h">,
  Intrinsic<[llvm_v8i16_ty],
            [llvm_v8i16_ty, llvm_i32_ty, llvm_v8i16_ty],
            [IntrNoMem]>;
def int_maxis_insve_w : GCCBuiltin<"__builtin_msa_insve_w">,
  Intrinsic<[llvm_v4i32_ty],
            [llvm_v4i32_ty, llvm_i32_ty, llvm_v4i32_ty],
            [IntrNoMem]>;
def int_maxis_insve_d : GCCBuiltin<"__builtin_msa_insve_d">,
  Intrinsic<[llvm_v2i64_ty],
            [llvm_v2i64_ty, llvm_i32_ty, llvm_v2i64_ty],
            [IntrNoMem]>;

def int_maxis_ld_b : GCCBuiltin<"__builtin_msa_ld_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_ptr_ty, llvm_i32_ty],
  [IntrReadMem, IntrArgMemOnly]>;
def int_maxis_ld_h : GCCBuiltin<"__builtin_msa_ld_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_ptr_ty, llvm_i32_ty],
  [IntrReadMem, IntrArgMemOnly]>;
def int_maxis_ld_w : GCCBuiltin<"__builtin_msa_ld_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_ptr_ty, llvm_i32_ty],
  [IntrReadMem, IntrArgMemOnly]>;
def int_maxis_ld_d : GCCBuiltin<"__builtin_msa_ld_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_ptr_ty, llvm_i32_ty],
  [IntrReadMem, IntrArgMemOnly]>;

def int_maxis_ldi_b : GCCBuiltin<"__builtin_msa_ldi_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_i32_ty], [IntrNoMem]>;
def int_maxis_ldi_h : GCCBuiltin<"__builtin_msa_ldi_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_i32_ty], [IntrNoMem]>;
def int_maxis_ldi_w : GCCBuiltin<"__builtin_msa_ldi_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_i32_ty], [IntrNoMem]>;
def int_maxis_ldi_d : GCCBuiltin<"__builtin_msa_ldi_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_i32_ty], [IntrNoMem]>;

// This instruction is part of the MSA spec but it does not share the
// __builtin_msa prefix because it operates on the GPR registers.
def int_maxis_lsa : GCCBuiltin<"__builtin_maxis_lsa">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty, llvm_i32_ty, llvm_i32_ty],
            [IntrNoMem]>;

def int_maxis_madd_q_h : GCCBuiltin<"__builtin_msa_madd_q_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty, llvm_v8i16_ty],
  [IntrNoMem]>;
def int_maxis_madd_q_w : GCCBuiltin<"__builtin_msa_madd_q_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty, llvm_v4i32_ty],
  [IntrNoMem]>;

def int_maxis_maddr_q_h : GCCBuiltin<"__builtin_msa_maddr_q_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty, llvm_v8i16_ty],
  [IntrNoMem]>;
def int_maxis_maddr_q_w : GCCBuiltin<"__builtin_msa_maddr_q_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty, llvm_v4i32_ty],
  [IntrNoMem]>;

def int_maxis_maddv_b : GCCBuiltin<"__builtin_msa_maddv_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_v16i8_ty],
  [IntrNoMem]>;
def int_maxis_maddv_h : GCCBuiltin<"__builtin_msa_maddv_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty, llvm_v8i16_ty],
  [IntrNoMem]>;
def int_maxis_maddv_w : GCCBuiltin<"__builtin_msa_maddv_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty, llvm_v4i32_ty],
  [IntrNoMem]>;
def int_maxis_maddv_d : GCCBuiltin<"__builtin_msa_maddv_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty, llvm_v2i64_ty],
  [IntrNoMem]>;

def int_maxis_max_a_b : GCCBuiltin<"__builtin_msa_max_a_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_max_a_h : GCCBuiltin<"__builtin_msa_max_a_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_max_a_w : GCCBuiltin<"__builtin_msa_max_a_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_max_a_d : GCCBuiltin<"__builtin_msa_max_a_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_max_s_b : GCCBuiltin<"__builtin_msa_max_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_max_s_h : GCCBuiltin<"__builtin_msa_max_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_max_s_w : GCCBuiltin<"__builtin_msa_max_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_max_s_d : GCCBuiltin<"__builtin_msa_max_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_max_u_b : GCCBuiltin<"__builtin_msa_max_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_max_u_h : GCCBuiltin<"__builtin_msa_max_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_max_u_w : GCCBuiltin<"__builtin_msa_max_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_max_u_d : GCCBuiltin<"__builtin_msa_max_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_maxi_s_b : GCCBuiltin<"__builtin_msa_maxi_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_maxi_s_h : GCCBuiltin<"__builtin_msa_maxi_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_maxi_s_w : GCCBuiltin<"__builtin_msa_maxi_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_maxi_s_d : GCCBuiltin<"__builtin_msa_maxi_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_maxi_u_b : GCCBuiltin<"__builtin_msa_maxi_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_maxi_u_h : GCCBuiltin<"__builtin_msa_maxi_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_maxi_u_w : GCCBuiltin<"__builtin_msa_maxi_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_maxi_u_d : GCCBuiltin<"__builtin_msa_maxi_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_min_a_b : GCCBuiltin<"__builtin_msa_min_a_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_min_a_h : GCCBuiltin<"__builtin_msa_min_a_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_min_a_w : GCCBuiltin<"__builtin_msa_min_a_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_min_a_d : GCCBuiltin<"__builtin_msa_min_a_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_min_s_b : GCCBuiltin<"__builtin_msa_min_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_min_s_h : GCCBuiltin<"__builtin_msa_min_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_min_s_w : GCCBuiltin<"__builtin_msa_min_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_min_s_d : GCCBuiltin<"__builtin_msa_min_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_min_u_b : GCCBuiltin<"__builtin_msa_min_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_min_u_h : GCCBuiltin<"__builtin_msa_min_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_min_u_w : GCCBuiltin<"__builtin_msa_min_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_min_u_d : GCCBuiltin<"__builtin_msa_min_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_mini_s_b : GCCBuiltin<"__builtin_msa_mini_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_mini_s_h : GCCBuiltin<"__builtin_msa_mini_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_mini_s_w : GCCBuiltin<"__builtin_msa_mini_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_mini_s_d : GCCBuiltin<"__builtin_msa_mini_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_mini_u_b : GCCBuiltin<"__builtin_msa_mini_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_mini_u_h : GCCBuiltin<"__builtin_msa_mini_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_mini_u_w : GCCBuiltin<"__builtin_msa_mini_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_mini_u_d : GCCBuiltin<"__builtin_msa_mini_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_mod_s_b : GCCBuiltin<"__builtin_msa_mod_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_mod_s_h : GCCBuiltin<"__builtin_msa_mod_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_mod_s_w : GCCBuiltin<"__builtin_msa_mod_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_mod_s_d : GCCBuiltin<"__builtin_msa_mod_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_mod_u_b : GCCBuiltin<"__builtin_msa_mod_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_mod_u_h : GCCBuiltin<"__builtin_msa_mod_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_mod_u_w : GCCBuiltin<"__builtin_msa_mod_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_mod_u_d : GCCBuiltin<"__builtin_msa_mod_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_move_v : GCCBuiltin<"__builtin_msa_move_v">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty], [IntrNoMem]>;

def int_maxis_msub_q_h : GCCBuiltin<"__builtin_msa_msub_q_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty, llvm_v8i16_ty],
  [IntrNoMem]>;
def int_maxis_msub_q_w : GCCBuiltin<"__builtin_msa_msub_q_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty, llvm_v4i32_ty],
  [IntrNoMem]>;

def int_maxis_msubr_q_h : GCCBuiltin<"__builtin_msa_msubr_q_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty, llvm_v8i16_ty],
  [IntrNoMem]>;
def int_maxis_msubr_q_w : GCCBuiltin<"__builtin_msa_msubr_q_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty, llvm_v4i32_ty],
  [IntrNoMem]>;

def int_maxis_msubv_b : GCCBuiltin<"__builtin_msa_msubv_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_v16i8_ty],
  [IntrNoMem]>;
def int_maxis_msubv_h : GCCBuiltin<"__builtin_msa_msubv_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty, llvm_v8i16_ty],
  [IntrNoMem]>;
def int_maxis_msubv_w : GCCBuiltin<"__builtin_msa_msubv_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty, llvm_v4i32_ty],
  [IntrNoMem]>;
def int_maxis_msubv_d : GCCBuiltin<"__builtin_msa_msubv_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty, llvm_v2i64_ty],
  [IntrNoMem]>;

def int_maxis_mul_q_h : GCCBuiltin<"__builtin_msa_mul_q_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_mul_q_w : GCCBuiltin<"__builtin_msa_mul_q_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;

def int_maxis_mulr_q_h : GCCBuiltin<"__builtin_msa_mulr_q_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_mulr_q_w : GCCBuiltin<"__builtin_msa_mulr_q_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;

def int_maxis_mulv_b : GCCBuiltin<"__builtin_msa_mulv_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_mulv_h : GCCBuiltin<"__builtin_msa_mulv_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_mulv_w : GCCBuiltin<"__builtin_msa_mulv_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_mulv_d : GCCBuiltin<"__builtin_msa_mulv_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_nloc_b : GCCBuiltin<"__builtin_msa_nloc_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_nloc_h : GCCBuiltin<"__builtin_msa_nloc_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_nloc_w : GCCBuiltin<"__builtin_msa_nloc_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_nloc_d : GCCBuiltin<"__builtin_msa_nloc_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_nlzc_b : GCCBuiltin<"__builtin_msa_nlzc_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_nlzc_h : GCCBuiltin<"__builtin_msa_nlzc_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_nlzc_w : GCCBuiltin<"__builtin_msa_nlzc_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_nlzc_d : GCCBuiltin<"__builtin_msa_nlzc_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_nor_v : GCCBuiltin<"__builtin_msa_nor_v">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;

def int_maxis_nori_b : GCCBuiltin<"__builtin_msa_nori_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_or_v : GCCBuiltin<"__builtin_msa_or_v">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;

def int_maxis_ori_b : GCCBuiltin<"__builtin_msa_ori_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_pckev_b : GCCBuiltin<"__builtin_msa_pckev_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_pckev_h : GCCBuiltin<"__builtin_msa_pckev_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_pckev_w : GCCBuiltin<"__builtin_msa_pckev_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_pckev_d : GCCBuiltin<"__builtin_msa_pckev_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_pckod_b : GCCBuiltin<"__builtin_msa_pckod_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_pckod_h : GCCBuiltin<"__builtin_msa_pckod_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_pckod_w : GCCBuiltin<"__builtin_msa_pckod_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_pckod_d : GCCBuiltin<"__builtin_msa_pckod_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_pcnt_b : GCCBuiltin<"__builtin_msa_pcnt_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_pcnt_h : GCCBuiltin<"__builtin_msa_pcnt_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_pcnt_w : GCCBuiltin<"__builtin_msa_pcnt_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_pcnt_d : GCCBuiltin<"__builtin_msa_pcnt_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_sat_s_b : GCCBuiltin<"__builtin_msa_sat_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_sat_s_h : GCCBuiltin<"__builtin_msa_sat_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_sat_s_w : GCCBuiltin<"__builtin_msa_sat_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_sat_s_d : GCCBuiltin<"__builtin_msa_sat_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_sat_u_b : GCCBuiltin<"__builtin_msa_sat_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_sat_u_h : GCCBuiltin<"__builtin_msa_sat_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_sat_u_w : GCCBuiltin<"__builtin_msa_sat_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_sat_u_d : GCCBuiltin<"__builtin_msa_sat_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_shf_b : GCCBuiltin<"__builtin_msa_shf_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_shf_h : GCCBuiltin<"__builtin_msa_shf_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_shf_w : GCCBuiltin<"__builtin_msa_shf_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_sld_b : GCCBuiltin<"__builtin_msa_sld_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_sld_h : GCCBuiltin<"__builtin_msa_sld_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_sld_w : GCCBuiltin<"__builtin_msa_sld_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_sld_d : GCCBuiltin<"__builtin_msa_sld_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_sldi_b : GCCBuiltin<"__builtin_msa_sldi_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_i32_ty],
            [IntrNoMem]>;
def int_maxis_sldi_h : GCCBuiltin<"__builtin_msa_sldi_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty, llvm_i32_ty],
            [IntrNoMem]>;
def int_maxis_sldi_w : GCCBuiltin<"__builtin_msa_sldi_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty, llvm_i32_ty],
            [IntrNoMem]>;
def int_maxis_sldi_d : GCCBuiltin<"__builtin_msa_sldi_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty, llvm_i32_ty],
            [IntrNoMem]>;

def int_maxis_sll_b : GCCBuiltin<"__builtin_msa_sll_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_sll_h : GCCBuiltin<"__builtin_msa_sll_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_sll_w : GCCBuiltin<"__builtin_msa_sll_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_sll_d : GCCBuiltin<"__builtin_msa_sll_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_slli_b : GCCBuiltin<"__builtin_msa_slli_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_slli_h : GCCBuiltin<"__builtin_msa_slli_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_slli_w : GCCBuiltin<"__builtin_msa_slli_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_slli_d : GCCBuiltin<"__builtin_msa_slli_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_splat_b : GCCBuiltin<"__builtin_msa_splat_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_splat_h : GCCBuiltin<"__builtin_msa_splat_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_splat_w : GCCBuiltin<"__builtin_msa_splat_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_splat_d : GCCBuiltin<"__builtin_msa_splat_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_splati_b : GCCBuiltin<"__builtin_msa_splati_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_splati_h : GCCBuiltin<"__builtin_msa_splati_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_splati_w : GCCBuiltin<"__builtin_msa_splati_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_splati_d : GCCBuiltin<"__builtin_msa_splati_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_sra_b : GCCBuiltin<"__builtin_msa_sra_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_sra_h : GCCBuiltin<"__builtin_msa_sra_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_sra_w : GCCBuiltin<"__builtin_msa_sra_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_sra_d : GCCBuiltin<"__builtin_msa_sra_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_srai_b : GCCBuiltin<"__builtin_msa_srai_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_srai_h : GCCBuiltin<"__builtin_msa_srai_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_srai_w : GCCBuiltin<"__builtin_msa_srai_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_srai_d : GCCBuiltin<"__builtin_msa_srai_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_srar_b : GCCBuiltin<"__builtin_msa_srar_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_srar_h : GCCBuiltin<"__builtin_msa_srar_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_srar_w : GCCBuiltin<"__builtin_msa_srar_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_srar_d : GCCBuiltin<"__builtin_msa_srar_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_srari_b : GCCBuiltin<"__builtin_msa_srari_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_srari_h : GCCBuiltin<"__builtin_msa_srari_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_srari_w : GCCBuiltin<"__builtin_msa_srari_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_srari_d : GCCBuiltin<"__builtin_msa_srari_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_srl_b : GCCBuiltin<"__builtin_msa_srl_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_srl_h : GCCBuiltin<"__builtin_msa_srl_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_srl_w : GCCBuiltin<"__builtin_msa_srl_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_srl_d : GCCBuiltin<"__builtin_msa_srl_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_srli_b : GCCBuiltin<"__builtin_msa_srli_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_srli_h : GCCBuiltin<"__builtin_msa_srli_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_srli_w : GCCBuiltin<"__builtin_msa_srli_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_srli_d : GCCBuiltin<"__builtin_msa_srli_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_srlr_b : GCCBuiltin<"__builtin_msa_srlr_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_srlr_h : GCCBuiltin<"__builtin_msa_srlr_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_srlr_w : GCCBuiltin<"__builtin_msa_srlr_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_srlr_d : GCCBuiltin<"__builtin_msa_srlr_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_srlri_b : GCCBuiltin<"__builtin_msa_srlri_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_srlri_h : GCCBuiltin<"__builtin_msa_srlri_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_srlri_w : GCCBuiltin<"__builtin_msa_srlri_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_srlri_d : GCCBuiltin<"__builtin_msa_srlri_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_st_b : GCCBuiltin<"__builtin_msa_st_b">,
  Intrinsic<[], [llvm_v16i8_ty, llvm_ptr_ty, llvm_i32_ty],
  [IntrArgMemOnly]>;
def int_maxis_st_h : GCCBuiltin<"__builtin_msa_st_h">,
  Intrinsic<[], [llvm_v8i16_ty, llvm_ptr_ty, llvm_i32_ty],
  [IntrArgMemOnly]>;
def int_maxis_st_w : GCCBuiltin<"__builtin_msa_st_w">,
  Intrinsic<[], [llvm_v4i32_ty, llvm_ptr_ty, llvm_i32_ty],
  [IntrArgMemOnly]>;
def int_maxis_st_d : GCCBuiltin<"__builtin_msa_st_d">,
  Intrinsic<[], [llvm_v2i64_ty, llvm_ptr_ty, llvm_i32_ty],
  [IntrArgMemOnly]>;

def int_maxis_subs_s_b : GCCBuiltin<"__builtin_msa_subs_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_subs_s_h : GCCBuiltin<"__builtin_msa_subs_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_subs_s_w : GCCBuiltin<"__builtin_msa_subs_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_subs_s_d : GCCBuiltin<"__builtin_msa_subs_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_subs_u_b : GCCBuiltin<"__builtin_msa_subs_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_subs_u_h : GCCBuiltin<"__builtin_msa_subs_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_subs_u_w : GCCBuiltin<"__builtin_msa_subs_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_subs_u_d : GCCBuiltin<"__builtin_msa_subs_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_subsus_u_b : GCCBuiltin<"__builtin_msa_subsus_u_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_subsus_u_h : GCCBuiltin<"__builtin_msa_subsus_u_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_subsus_u_w : GCCBuiltin<"__builtin_msa_subsus_u_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_subsus_u_d : GCCBuiltin<"__builtin_msa_subsus_u_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_subsuu_s_b : GCCBuiltin<"__builtin_msa_subsuu_s_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_subsuu_s_h : GCCBuiltin<"__builtin_msa_subsuu_s_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_subsuu_s_w : GCCBuiltin<"__builtin_msa_subsuu_s_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_subsuu_s_d : GCCBuiltin<"__builtin_msa_subsuu_s_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_subv_b : GCCBuiltin<"__builtin_msa_subv_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;
def int_maxis_subv_h : GCCBuiltin<"__builtin_msa_subv_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
def int_maxis_subv_w : GCCBuiltin<"__builtin_msa_subv_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
def int_maxis_subv_d : GCCBuiltin<"__builtin_msa_subv_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty], [IntrNoMem]>;

def int_maxis_subvi_b : GCCBuiltin<"__builtin_msa_subvi_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_subvi_h : GCCBuiltin<"__builtin_msa_subvi_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_subvi_w : GCCBuiltin<"__builtin_msa_subvi_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_i32_ty], [IntrNoMem]>;
def int_maxis_subvi_d : GCCBuiltin<"__builtin_msa_subvi_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_i32_ty], [IntrNoMem]>;

def int_maxis_vshf_b : GCCBuiltin<"__builtin_msa_vshf_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty, llvm_v16i8_ty],
            [IntrNoMem]>;
def int_maxis_vshf_h : GCCBuiltin<"__builtin_msa_vshf_h">,
  Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty, llvm_v8i16_ty],
            [IntrNoMem]>;
def int_maxis_vshf_w : GCCBuiltin<"__builtin_msa_vshf_w">,
  Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty, llvm_v4i32_ty],
            [IntrNoMem]>;
def int_maxis_vshf_d : GCCBuiltin<"__builtin_msa_vshf_d">,
  Intrinsic<[llvm_v2i64_ty], [llvm_v2i64_ty, llvm_v2i64_ty, llvm_v2i64_ty],
            [IntrNoMem]>;

def int_maxis_xor_v : GCCBuiltin<"__builtin_msa_xor_v">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty], [IntrNoMem]>;

def int_maxis_xori_b : GCCBuiltin<"__builtin_msa_xori_b">,
  Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_i32_ty], [IntrNoMem]>;
}
