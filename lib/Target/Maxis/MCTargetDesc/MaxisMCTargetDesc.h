//===-- MaxisMCTargetDesc.h - Maxis Target Descriptions -----------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides Maxis specific target descriptions.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MAXIS_MCTARGETDESC_MAXISMCTARGETDESC_H
#define LLVM_LIB_TARGET_MAXIS_MCTARGETDESC_MAXISMCTARGETDESC_H

#include "llvm/Support/DataTypes.h"

#include <memory>

namespace llvm {
class MCAsmBackend;
class MCCodeEmitter;
class MCContext;
class MCInstrInfo;
class MCObjectWriter;
class MCRegisterInfo;
class MCSubtargetInfo;
class MCTargetOptions;
class StringRef;
class Target;
class Triple;
class raw_ostream;
class raw_pwrite_stream;

Target &getTheMaxisTarget();
Target &getTheMaxiselTarget();
Target &getTheMaxis64Target();
Target &getTheMaxis64elTarget();

MCCodeEmitter *createMaxisMCCodeEmitterEB(const MCInstrInfo &MCII,
                                         const MCRegisterInfo &MRI,
                                         MCContext &Ctx);
MCCodeEmitter *createMaxisMCCodeEmitterEL(const MCInstrInfo &MCII,
                                         const MCRegisterInfo &MRI,
                                         MCContext &Ctx);

MCAsmBackend *createMaxisAsmBackend(const Target &T, const MCSubtargetInfo &STI,
                                   const MCRegisterInfo &MRI,
                                   const MCTargetOptions &Options);

std::unique_ptr<MCObjectWriter>
createMaxisELFObjectWriter(raw_pwrite_stream &OS, const Triple &TT, bool IsN32);

namespace MAXIS_MC {
StringRef selectMaxisCPU(const Triple &TT, StringRef CPU);
}

} // End llvm namespace

// Defines symbolic names for Maxis registers.  This defines a mapping from
// register name to register number.
#define GET_REGINFO_ENUM
#include "MaxisGenRegisterInfo.inc"

// Defines symbolic names for the Maxis instructions.
#define GET_INSTRINFO_ENUM
#include "MaxisGenInstrInfo.inc"

#define GET_SUBTARGETINFO_ENUM
#include "MaxisGenSubtargetInfo.inc"

#endif
