//===-- MaxisNaClELFStreamer.cpp - ELF Object Output for Maxis NaCl ---------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements MCELFStreamer for Maxis NaCl.  It emits .o object files
// as required by NaCl's SFI sandbox.  It inserts address-masking instructions
// before dangerous control-flow and memory access instructions.  It inserts
// address-masking instructions after instructions that change the stack
// pointer.  It ensures that the mask and the dangerous instruction are always
// emitted in the same bundle.  It aligns call + branch delay to the bundle end,
// so that return address is always aligned to the start of next bundle.
//
//===----------------------------------------------------------------------===//

#include "Maxis.h"
#include "MaxisELFStreamer.h"
#include "MaxisMCNaCl.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCInst.h"
#include "llvm/Support/ErrorHandling.h"
#include <cassert>

using namespace llvm;

#define DEBUG_TYPE "maxis-mc-nacl"

namespace {

const unsigned IndirectBranchMaskReg = Maxis::T6;
const unsigned LoadStoreStackMaskReg = Maxis::T7;

/// Extend the generic MCELFStreamer class so that it can mask dangerous
/// instructions.

class MaxisNaClELFStreamer : public MaxisELFStreamer {
public:
  MaxisNaClELFStreamer(MCContext &Context, std::unique_ptr<MCAsmBackend> TAB,
                      raw_pwrite_stream &OS,
                      std::unique_ptr<MCCodeEmitter> Emitter)
      : MaxisELFStreamer(Context, std::move(TAB), OS, std::move(Emitter)) {}

  ~MaxisNaClELFStreamer() override = default;

private:
  // Whether we started the sandboxing sequence for calls.  Calls are bundled
  // with branch delays and aligned to the bundle end.
  bool PendingCall = false;

  bool isIndirectJump(const MCInst &MI) {
    if (MI.getOpcode() == Maxis::JALR) {
      // MAXIS32r6/MAXIS64r6 doesn't have a JR instruction and uses JALR instead.
      // JALR is an indirect branch if the link register is $0.
      assert(MI.getOperand(0).isReg());
      return MI.getOperand(0).getReg() == Maxis::ZERO;
    }
    //    return MI.getOpcode() == Maxis::JR;
    return false;
  }

  bool isStackPointerFirstOperand(const MCInst &MI) {
    return (MI.getNumOperands() > 0 && MI.getOperand(0).isReg()
            && MI.getOperand(0).getReg() == Maxis::SP);
  }

  bool isCall(const MCInst &MI, bool *IsIndirectCall) {
    unsigned Opcode = MI.getOpcode();

    *IsIndirectCall = false;

    switch (Opcode) {
    default:
      return false;

    case Maxis::JAL:
    case Maxis::BAL:
    case Maxis::BAL_BR:
    case Maxis::BLTZAL:
    case Maxis::BGEZAL:
      return true;

    case Maxis::JALR:
      // JALR is only a call if the link register is not $0. Otherwise it's an
      // indirect branch.
      assert(MI.getOperand(0).isReg());
      if (MI.getOperand(0).getReg() == Maxis::ZERO)
        return false;

      *IsIndirectCall = true;
      return true;
    }
  }

  void emitMask(unsigned AddrReg, unsigned MaskReg,
                const MCSubtargetInfo &STI) {
    MCInst MaskInst;
    MaskInst.setOpcode(Maxis::AND);
    MaskInst.addOperand(MCOperand::createReg(AddrReg));
    MaskInst.addOperand(MCOperand::createReg(AddrReg));
    MaskInst.addOperand(MCOperand::createReg(MaskReg));
    MaxisELFStreamer::EmitInstruction(MaskInst, STI);
  }

  // Sandbox indirect branch or return instruction by inserting mask operation
  // before it.
  void sandboxIndirectJump(const MCInst &MI, const MCSubtargetInfo &STI) {
    unsigned AddrReg = MI.getOperand(0).getReg();

    EmitBundleLock(false);
    emitMask(AddrReg, IndirectBranchMaskReg, STI);
    MaxisELFStreamer::EmitInstruction(MI, STI);
    EmitBundleUnlock();
  }

  // Sandbox memory access or SP change.  Insert mask operation before and/or
  // after the instruction.
  void sandboxLoadStoreStackChange(const MCInst &MI, unsigned AddrIdx,
                                   const MCSubtargetInfo &STI, bool MaskBefore,
                                   bool MaskAfter) {
    EmitBundleLock(false);
    if (MaskBefore) {
      // Sandbox memory access.
      unsigned BaseReg = MI.getOperand(AddrIdx).getReg();
      emitMask(BaseReg, LoadStoreStackMaskReg, STI);
    }
    MaxisELFStreamer::EmitInstruction(MI, STI);
    if (MaskAfter) {
      // Sandbox SP change.
      unsigned SPReg = MI.getOperand(0).getReg();
      assert((Maxis::SP == SPReg) && "Unexpected stack-pointer register.");
      emitMask(SPReg, LoadStoreStackMaskReg, STI);
    }
    EmitBundleUnlock();
  }

public:
  /// This function is the one used to emit instruction data into the ELF
  /// streamer.  We override it to mask dangerous instructions.
  void EmitInstruction(const MCInst &Inst, const MCSubtargetInfo &STI,
                       bool) override {
    // Sandbox indirect jumps.
    if (isIndirectJump(Inst)) {
      if (PendingCall)
        report_fatal_error("Dangerous instruction in branch delay slot!");
      sandboxIndirectJump(Inst, STI);
      return;
    }

    // Sandbox loads, stores and SP changes.
    unsigned AddrIdx;
    bool IsStore;
    bool IsMemAccess = isMaxisBasePlusOffsetMemoryAccess(Inst.getOpcode(), &AddrIdx,
                                                    &IsStore);
    bool IsSPFirstOperand = isStackPointerFirstOperand(Inst);
    if (IsMemAccess || IsSPFirstOperand) {
      bool MaskBefore = (IsMemAccess
                         && baseMaxisRegNeedsLoadStoreMask(Inst.getOperand(AddrIdx)
                                                          .getReg()));
      bool MaskAfter = IsSPFirstOperand && !IsStore;
      if (MaskBefore || MaskAfter) {
        if (PendingCall)
          report_fatal_error("Dangerous instruction in branch delay slot!");
        sandboxLoadStoreStackChange(Inst, AddrIdx, STI, MaskBefore, MaskAfter);
        return;
      }
      // fallthrough
    }

    // Sandbox calls by aligning call and branch delay to the bundle end.
    // For indirect calls, emit the mask before the call.
    bool IsIndirectCall;
    if (isCall(Inst, &IsIndirectCall)) {
      if (PendingCall)
        report_fatal_error("Dangerous instruction in branch delay slot!");

      // Start the sandboxing sequence by emitting call.
      EmitBundleLock(true);
      if (IsIndirectCall) {
        unsigned TargetReg = Inst.getOperand(1).getReg();
        emitMask(TargetReg, IndirectBranchMaskReg, STI);
      }
      MaxisELFStreamer::EmitInstruction(Inst, STI);
      PendingCall = true;
      return;
    }
    if (PendingCall) {
      // Finish the sandboxing sequence by emitting branch delay.
      MaxisELFStreamer::EmitInstruction(Inst, STI);
      EmitBundleUnlock();
      PendingCall = false;
      return;
    }

    // None of the sandboxing applies, just emit the instruction.
    MaxisELFStreamer::EmitInstruction(Inst, STI);
  }
};

} // end anonymous namespace

namespace llvm {

bool isMaxisBasePlusOffsetMemoryAccess(unsigned Opcode, unsigned *AddrIdx,
                                  bool *IsStore) {
  if (IsStore)
    *IsStore = false;

  switch (Opcode) {
  default:
    return false;

  // Load instructions with base address register in position 1.
  case Maxis::LB:
  case Maxis::LBu:
  case Maxis::LH:
  case Maxis::LHu:
  case Maxis::LW:
  case Maxis::LWC1:
  case Maxis::LDC1:
  case Maxis::LL:
  case Maxis::LL_R6:
  case Maxis::LWL:
  case Maxis::LWR:
    *AddrIdx = 1;
    return true;

  // Store instructions with base address register in position 1.
  case Maxis::SB:
  case Maxis::SH:
  case Maxis::SW:
  case Maxis::SWC1:
  case Maxis::SDC1:
  case Maxis::SWL:
  case Maxis::SWR:
    *AddrIdx = 1;
    if (IsStore)
      *IsStore = true;
    return true;

  // Store instructions with base address register in position 2.
  case Maxis::SC:
  case Maxis::SC_R6:
    *AddrIdx = 2;
    if (IsStore)
      *IsStore = true;
    return true;
  }
}

bool baseMaxisRegNeedsLoadStoreMask(unsigned Reg) {
  // The contents of SP and thread pointer register do not require masking.
  return Reg != Maxis::SP && Reg != Maxis::T8;
}

MCELFStreamer *createMaxisNaClELFStreamer(MCContext &Context,
                                         std::unique_ptr<MCAsmBackend> TAB,
                                         raw_pwrite_stream &OS,
                                         std::unique_ptr<MCCodeEmitter> Emitter,
                                         bool RelaxAll) {
  MaxisNaClELFStreamer *S =
      new MaxisNaClELFStreamer(Context, std::move(TAB), OS, std::move(Emitter));
  if (RelaxAll)
    S->getAssembler().setRelaxAll(true);

  // Set bundle-alignment as required by the NaCl ABI for the target.
  S->EmitBundleAlignMode(MAXIS_NACL_BUNDLE_ALIGN);

  return S;
}

} // end namespace llvm
