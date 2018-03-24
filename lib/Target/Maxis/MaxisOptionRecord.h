//===- MaxisOptionRecord.h - Abstraction for storing information -*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// MaxisOptionRecord - Abstraction for storing arbitrary information in
// ELF files. Arbitrary information (e.g. register usage) can be stored in Maxis
// specific ELF sections like .Maxis.options. Specific records should subclass
// MaxisOptionRecord and provide an implementation to EmitMaxisOptionRecord which
// basically just dumps the information into an ELF section. More information
// about .Maxis.option can be found in the SysV ABI and the 64-bit ELF Object
// specification.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MAXIS_MAXISOPTIONRECORD_H
#define LLVM_LIB_TARGET_MAXIS_MAXISOPTIONRECORD_H

#include "MCTargetDesc/MaxisMCTargetDesc.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCRegisterInfo.h"
#include <cstdint>

namespace llvm {

class MaxisELFStreamer;

class MaxisOptionRecord {
public:
  virtual ~MaxisOptionRecord() = default;

  virtual void EmitMaxisOptionRecord() = 0;
};

class MaxisRegInfoRecord : public MaxisOptionRecord {
public:
  MaxisRegInfoRecord(MaxisELFStreamer *S, MCContext &Context)
      : Streamer(S), Context(Context) {
    ri_gprmask = 0;
    ri_cprmask[0] = ri_cprmask[1] = ri_cprmask[2] = ri_cprmask[3] = 0;
    ri_gp_value = 0;

    const MCRegisterInfo *TRI = Context.getRegisterInfo();
    GPR32RegClass = &(TRI->getRegClass(Maxis::GPR32RegClassID));
    GPR64RegClass = &(TRI->getRegClass(Maxis::GPR64RegClassID));
    FGR32RegClass = &(TRI->getRegClass(Maxis::FGR32RegClassID));
    FGR64RegClass = &(TRI->getRegClass(Maxis::FGR64RegClassID));
    AFGR64RegClass = &(TRI->getRegClass(Maxis::AFGR64RegClassID));
    MSA128BRegClass = &(TRI->getRegClass(Maxis::MSA128BRegClassID));
    COP0RegClass = &(TRI->getRegClass(Maxis::COP0RegClassID));
    COP2RegClass = &(TRI->getRegClass(Maxis::COP2RegClassID));
    COP3RegClass = &(TRI->getRegClass(Maxis::COP3RegClassID));
  }

  ~MaxisRegInfoRecord() override = default;

  void EmitMaxisOptionRecord() override;
  void SetPhysRegUsed(unsigned Reg, const MCRegisterInfo *MCRegInfo);

private:
  MaxisELFStreamer *Streamer;
  MCContext &Context;
  const MCRegisterClass *GPR32RegClass;
  const MCRegisterClass *GPR64RegClass;
  const MCRegisterClass *FGR32RegClass;
  const MCRegisterClass *FGR64RegClass;
  const MCRegisterClass *AFGR64RegClass;
  const MCRegisterClass *MSA128BRegClass;
  const MCRegisterClass *COP0RegClass;
  const MCRegisterClass *COP2RegClass;
  const MCRegisterClass *COP3RegClass;
  uint32_t ri_gprmask;
  uint32_t ri_cprmask[4];
  int64_t ri_gp_value;
};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_MAXIS_MAXISOPTIONRECORD_H
