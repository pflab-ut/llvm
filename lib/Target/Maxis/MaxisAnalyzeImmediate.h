//===- MaxisAnalyzeImmediate.h - Analyze Immediates -------------*- C++ -*--===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MAXIS_MAXISANALYZEIMMEDIATE_H
#define LLVM_LIB_TARGET_MAXIS_MAXISANALYZEIMMEDIATE_H

#include "llvm/ADT/SmallVector.h"
#include <cstdint>

namespace llvm {

  class MaxisAnalyzeImmediate {
  public:
    struct Inst {
      unsigned Opc, ImmOpnd;

      Inst(unsigned Opc, unsigned ImmOpnd);
    };
    using InstSeq = SmallVector<Inst, 7>;

    /// Analyze - Get an instruction sequence to load immediate Imm. The last
    /// instruction in the sequence must be an ADDi if LastInstrIsADDi is
    /// true;
    const InstSeq &Analyze(uint64_t Imm, unsigned Size, bool LastInstrIsADDi);

  private:
    using InstSeqLs = SmallVector<InstSeq, 5>;

    /// AddInstr - Add I to all instruction sequences in SeqLs.
    void AddInstr(InstSeqLs &SeqLs, const Inst &I);

    /// GetInstSeqLsADDi - Get instruction sequences which end with an ADDi to
    /// load immediate Imm
    void GetInstSeqLsADDi(uint64_t Imm, unsigned RemSize, InstSeqLs &SeqLs);

    /// GetInstSeqLsORi - Get instrutcion sequences which end with an ORi to
    /// load immediate Imm
    void GetInstSeqLsORi(uint64_t Imm, unsigned RemSize, InstSeqLs &SeqLs);

    /// GetInstSeqLsSLLi - Get instruction sequences which end with a SLLi to
    /// load immediate Imm
    void GetInstSeqLsSLLi(uint64_t Imm, unsigned RemSize, InstSeqLs &SeqLs);

    /// GetInstSeqLs - Get instruction sequences to load immediate Imm.
    void GetInstSeqLs(uint64_t Imm, unsigned RemSize, InstSeqLs &SeqLs);

    /// ReplaceADDiSLLiWithCATi - Replace an ADDi & SLLi pair with a CATi.
    void ReplaceADDiSLLiWithCATi(InstSeq &Seq);

    /// GetShortestSeq - Find the shortest instruction sequence in SeqLs and
    /// return it in Insts.
    void GetShortestSeq(InstSeqLs &SeqLs, InstSeq &Insts);

    unsigned Size;
    unsigned ADDi, ORi, SLLi, CATi;
    InstSeq Insts;
  };

} // end namespace llvm

#endif // LLVM_LIB_TARGET_MAXIS_MAXISANALYZEIMMEDIATE_H
