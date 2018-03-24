//===-- MaxisMCNaCl.h - NaCl-related declarations --------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MAXIS_MCTARGETDESC_MAXISMCNACL_H
#define LLVM_LIB_TARGET_MAXIS_MCTARGETDESC_MAXISMCNACL_H

#include "llvm/MC/MCELFStreamer.h"

namespace llvm {

// Log2 of the NaCl MAXIS sandbox's instruction bundle size.
static const unsigned MAXIS_NACL_BUNDLE_ALIGN = 4u;

bool isBasePlusOffsetMemoryAccess(unsigned Opcode, unsigned *AddrIdx,
                                  bool *IsStore = nullptr);
bool baseRegNeedsLoadStoreMask(unsigned Reg);

// This function creates an MCELFStreamer for Maxis NaCl.
MCELFStreamer *createMaxisNaClELFStreamer(MCContext &Context,
                                         std::unique_ptr<MCAsmBackend> TAB,
                                         raw_pwrite_stream &OS,
                                         std::unique_ptr<MCCodeEmitter> Emitter,
                                         bool RelaxAll);
}

#endif
