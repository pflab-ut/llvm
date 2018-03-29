//===-- MaxisFixupKinds.h - Maxis Specific Fixup Entries ----------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MAXIS_MCTARGETDESC_MAXISFIXUPKINDS_H
#define LLVM_LIB_TARGET_MAXIS_MCTARGETDESC_MAXISFIXUPKINDS_H

#include "llvm/MC/MCFixup.h"

namespace llvm {
namespace Maxis {
  // Although most of the current fixup types reflect a unique relocation
  // one can have multiple fixup types for a given relocation and thus need
  // to be uniquely named.
  //
  // This table *must* be in the same order of
  // MCFixupKindInfo Infos[Maxis::NumTargetFixupKinds]
  // in MaxisAsmBackend.cpp.
  //
  enum Fixups {
    // Branch fixups resulting in R_MAXIS_NONE.
    fixup_Maxis_NONE = FirstTargetFixupKind,

    // Branch fixups resulting in R_MAXIS_16.
    fixup_Maxis_16,

    // Pure 32 bit data fixup resulting in - R_MAXIS_32.
    fixup_Maxis_32,

    // Full 32 bit data relative data fixup resulting in - R_MAXIS_REL32.
    fixup_Maxis_REL32,

    // Jump 26 bit fixup resulting in - R_MAXIS_21.
    fixup_Maxis_21,

    // Pure upper 16 bit fixup resulting in - R_MAXIS_HI16.
    fixup_Maxis_HI16,

    // Pure lower 16 bit fixup resulting in - R_MAXIS_LO16.
    fixup_Maxis_LO16,

    // 16 bit fixup for GP offest resulting in - R_MAXIS_GPREL16.
    fixup_Maxis_GPREL16,

    // 16 bit literal fixup resulting in - R_MAXIS_LITERAL.
    fixup_Maxis_LITERAL,

    // Symbol fixup resulting in - R_MAXIS_GOT16.
    fixup_Maxis_GOT,

    // PC relative branch fixup resulting in - R_MAXIS_PC16.
    fixup_Maxis_PC16,

    // resulting in - R_MAXIS_CALL16.
    fixup_Maxis_CALL16,

    // resulting in - R_MAXIS_GPREL32.
    fixup_Maxis_GPREL32,

    // resulting in - R_MAXIS_SHIFT5.
    fixup_Maxis_SHIFT5,

    // resulting in - R_MAXIS_SHIFT6.
    fixup_Maxis_SHIFT6,

    // Pure 64 bit data fixup resulting in - R_MAXIS_64.
    fixup_Maxis_64,

    // resulting in - R_MAXIS_TLS_GD.
    fixup_Maxis_TLSGD,

    // resulting in - R_MAXIS_TLS_GOTTPREL.
    fixup_Maxis_GOTTPREL,

    // resulting in - R_MAXIS_TLS_TPREL_HI16.
    fixup_Maxis_TPREL_HI,

    // resulting in - R_MAXIS_TLS_TPREL_LO16.
    fixup_Maxis_TPREL_LO,

    // resulting in - R_MAXIS_TLS_LDM.
    fixup_Maxis_TLSLDM,

    // resulting in - R_MAXIS_TLS_DTPREL_HI16.
    fixup_Maxis_DTPREL_HI,

    // resulting in - R_MAXIS_TLS_DTPREL_LO16.
    fixup_Maxis_DTPREL_LO,

    // PC relative branch fixup resulting in - R_MAXIS_PC16
    fixup_Maxis_Branch_PCRel,

    // resulting in - R_MAXIS_GPREL16/R_MAXIS_SUB/R_MAXIS_HI16
    fixup_Maxis_GPOFF_HI,

    // resulting in - R_MAXIS_GPREL16/R_MAXIS_SUB/R_MAXIS_LO16
    fixup_Maxis_GPOFF_LO,

    // resulting in - R_MAXIS_PAGE
    fixup_Maxis_GOT_PAGE,

    // resulting in - R_MAXIS_GOT_OFST
    fixup_Maxis_GOT_OFST,

    // resulting in - R_MAXIS_GOT_DISP
    fixup_Maxis_GOT_DISP,

    // resulting in - R_MAXIS_GOT_HIGHER
    fixup_Maxis_HIGHER,

    // resulting in - R_MAXIS_HIGHEST
    fixup_Maxis_HIGHEST,

    // resulting in - R_MAXIS_GOT_HI16
    fixup_Maxis_GOT_HI16,

    // resulting in - R_MAXIS_GOT_LO16
    fixup_Maxis_GOT_LO16,

    // resulting in - R_MAXIS_CALL_HI16
    fixup_Maxis_CALL_HI16,

    // resulting in - R_MAXIS_CALL_LO16
    fixup_Maxis_CALL_LO16,

    // resulting in - R_MAXIS_PC18_S3
    fixup_MAXIS_PC18_S3,

    // resulting in - R_MAXIS_PC19_S2
    fixup_MAXIS_PC19_S2,

    // resulting in - R_MAXIS_PC21_S2
    fixup_MAXIS_PC21_S2,

    // resulting in - R_MAXIS_PC26_S2
    fixup_MAXIS_PC26_S2,

    // resulting in - R_MAXIS_PCHI16
    fixup_MAXIS_PCHI16,

    // resulting in - R_MAXIS_PCLO16
    fixup_MAXIS_PCLO16,

    // resulting in - R_MICROMAXIS_26_S1
    fixup_MICROMAXIS_26_S1,

    // resulting in - R_MICROMAXIS_HI16
    fixup_MICROMAXIS_HI16,

    // resulting in - R_MICROMAXIS_LO16
    fixup_MICROMAXIS_LO16,

    // resulting in - R_MICROMAXIS_GOT16
    fixup_MICROMAXIS_GOT16,

    // resulting in - R_MICROMAXIS_PC7_S1
    fixup_MICROMAXIS_PC7_S1,

    // resulting in - R_MICROMAXIS_PC10_S1
    fixup_MICROMAXIS_PC10_S1,

    // resulting in - R_MICROMAXIS_PC16_S1
    fixup_MICROMAXIS_PC16_S1,

    // resulting in - R_MICROMAXIS_PC26_S1
    fixup_MICROMAXIS_PC26_S1,

    // resulting in - R_MICROMAXIS_PC19_S2
    fixup_MICROMAXIS_PC19_S2,

    // resulting in - R_MICROMAXIS_PC18_S3
    fixup_MICROMAXIS_PC18_S3,

    // resulting in - R_MICROMAXIS_PC21_S1
    fixup_MICROMAXIS_PC21_S1,

    // resulting in - R_MICROMAXIS_CALL16
    fixup_MICROMAXIS_CALL16,

    // resulting in - R_MICROMAXIS_GOT_DISP
    fixup_MICROMAXIS_GOT_DISP,

    // resulting in - R_MICROMAXIS_GOT_PAGE
    fixup_MICROMAXIS_GOT_PAGE,

    // resulting in - R_MICROMAXIS_GOT_OFST
    fixup_MICROMAXIS_GOT_OFST,

    // resulting in - R_MICROMAXIS_TLS_GD
    fixup_MICROMAXIS_TLS_GD,

    // resulting in - R_MICROMAXIS_TLS_LDM
    fixup_MICROMAXIS_TLS_LDM,

    // resulting in - R_MICROMAXIS_TLS_DTPREL_HI16
    fixup_MICROMAXIS_TLS_DTPREL_HI16,

    // resulting in - R_MICROMAXIS_TLS_DTPREL_LO16
    fixup_MICROMAXIS_TLS_DTPREL_LO16,

    // resulting in - R_MICROMAXIS_TLS_GOTTPREL.
    fixup_MICROMAXIS_GOTTPREL,

    // resulting in - R_MICROMAXIS_TLS_TPREL_HI16
    fixup_MICROMAXIS_TLS_TPREL_HI16,

    // resulting in - R_MICROMAXIS_TLS_TPREL_LO16
    fixup_MICROMAXIS_TLS_TPREL_LO16,

    // resulting in - R_MAXIS_SUB/R_MICROMAXIS_SUB
    fixup_Maxis_SUB,
    fixup_MICROMAXIS_SUB,

    // Marker
    LastTargetFixupKind,
    NumTargetFixupKinds = LastTargetFixupKind - FirstTargetFixupKind
  };
} // namespace Maxis
} // namespace llvm


#endif
