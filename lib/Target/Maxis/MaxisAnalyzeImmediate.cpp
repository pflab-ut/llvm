//===- MaxisAnalyzeImmediate.cpp - Analyze Immediates ----------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MaxisAnalyzeImmediate.h"
#include "Maxis.h"
#include "llvm/Support/MathExtras.h"
#include <cassert>
#include <cstdint>
#include <iterator>

using namespace llvm;

MaxisAnalyzeImmediate::Inst::Inst(unsigned O, unsigned I) : Opc(O), ImmOpnd(I) {}

// Add I to the instruction sequences.
void MaxisAnalyzeImmediate::AddInstr(InstSeqLs &SeqLs, const Inst &I) {
  // Add an instruction seqeunce consisting of just I.
  if (SeqLs.empty()) {
    SeqLs.push_back(InstSeq(1, I));
    return;
  }

  for (InstSeqLs::iterator Iter = SeqLs.begin(); Iter != SeqLs.end(); ++Iter)
    Iter->push_back(I);
}

void MaxisAnalyzeImmediate::GetInstSeqLsADDi(uint64_t Imm, unsigned RemSize,
                                             InstSeqLs &SeqLs) {
  GetInstSeqLs((Imm + 0x8000ULL) & 0xffffffffffff0000ULL, RemSize, SeqLs);
  AddInstr(SeqLs, Inst(ADDi, Imm & 0xffffULL));
}

void MaxisAnalyzeImmediate::GetInstSeqLsORi(uint64_t Imm, unsigned RemSize,
                                           InstSeqLs &SeqLs) {
  GetInstSeqLs(Imm & 0xffffffffffff0000ULL, RemSize, SeqLs);
  AddInstr(SeqLs, Inst(ORi, Imm & 0xffffULL));
}

void MaxisAnalyzeImmediate::GetInstSeqLsSLL(uint64_t Imm, unsigned RemSize,
                                           InstSeqLs &SeqLs) {
  unsigned Shamt = countTrailingZeros(Imm);
  GetInstSeqLs(Imm >> Shamt, RemSize - Shamt, SeqLs);
  AddInstr(SeqLs, Inst(SLL, Shamt));
}

void MaxisAnalyzeImmediate::GetInstSeqLs(uint64_t Imm, unsigned RemSize,
                                        InstSeqLs &SeqLs) {
  uint64_t MaskedImm = Imm & (0xffffffffffffffffULL >> (64 - Size));

  // Do nothing if Imm is 0.
  if (!MaskedImm)
    return;

  // A single ADDi will do if RemSize <= 16.
  if (RemSize <= 16) {
    AddInstr(SeqLs, Inst(ADDi, MaskedImm));
    return;
  }

  // Shift if the lower 16-bit is cleared.
  if (!(Imm & 0xffff)) {
    GetInstSeqLsSLL(Imm, RemSize, SeqLs);
    return;
  }

  GetInstSeqLsADDi(Imm, RemSize, SeqLs);

  // If bit 15 is cleared, it doesn't make a difference whether the last
  // instruction is an ADDi or ORi. In that case, do not call GetInstSeqLsORi.
  if (Imm & 0x8000) {
    InstSeqLs SeqLsORi;
    GetInstSeqLsORi(Imm, RemSize, SeqLsORi);
    SeqLs.append(std::make_move_iterator(SeqLsORi.begin()),
                 std::make_move_iterator(SeqLsORi.end()));
  }
}

// Replace a ADDi & SLL pair with a LUi.
// e.g. the following two instructions
//  ADDi 0x0111
//  SLL 18
// are replaced with
//  LUi 0x444
void MaxisAnalyzeImmediate::ReplaceADDiSLLWithLUi(InstSeq &Seq) {
  // Check if the first two instructions are ADDi and SLL and the shift amount
  // is at least 16.
  if ((Seq.size() < 2) || (Seq[0].Opc != ADDi) ||
      (Seq[1].Opc != SLL) || (Seq[1].ImmOpnd < 16))
    return;

  // Sign-extend and shift operand of ADDi and see if it still fits in 16-bit.
  int64_t Imm = SignExtend64<16>(Seq[0].ImmOpnd);
  int64_t ShiftedImm = (uint64_t)Imm << (Seq[1].ImmOpnd - 16);

  if (!isInt<16>(ShiftedImm))
    return;

  // Replace the first instruction and erase the second.
  Seq[0].Opc = LUi;
  Seq[0].ImmOpnd = (unsigned)(ShiftedImm & 0xffff);
  Seq.erase(Seq.begin() + 1);
}

void MaxisAnalyzeImmediate::GetShortestSeq(InstSeqLs &SeqLs, InstSeq &Insts) {
  InstSeqLs::iterator ShortestSeq = SeqLs.end();
  // The length of an instruction sequence is at most 7.
  unsigned ShortestLength = 8;

  for (InstSeqLs::iterator S = SeqLs.begin(); S != SeqLs.end(); ++S) {
    ReplaceADDiSLLWithLUi(*S);
    assert(S->size() <= 7);

    if (S->size() < ShortestLength) {
      ShortestSeq = S;
      ShortestLength = S->size();
    }
  }

  Insts.clear();
  Insts.append(ShortestSeq->begin(), ShortestSeq->end());
}

const MaxisAnalyzeImmediate::InstSeq
&MaxisAnalyzeImmediate::Analyze(uint64_t Imm, unsigned Size,
                               bool LastInstrIsADDi) {
  this->Size = Size;

  if (Size == 32) {
    ADDi = Maxis::ADDi;
    ORi = Maxis::ORi;
    SLL = Maxis::SLL;
    LUi = Maxis::LUi;
  } else {
    ADDi = Maxis::DADDi;
    ORi = Maxis::ORi64;
    SLL = Maxis::DSLL;
    LUi = Maxis::LUi64;
  }

  InstSeqLs SeqLs;

  // Get the list of instruction sequences.
  if (LastInstrIsADDi | !Imm)
    GetInstSeqLsADDi(Imm, Size, SeqLs);
  else
    GetInstSeqLs(Imm, Size, SeqLs);

  // Set Insts to the shortest instruction sequence.
  GetShortestSeq(SeqLs, Insts);

  return Insts;
}
