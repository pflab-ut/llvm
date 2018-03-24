//===-- RuntimeDyldELFMaxis.h ---- ELF/Maxis specific code. -------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_EXECUTIONENGINE_RUNTIMEDYLD_TARGETS_RUNTIMEDYLDELFMAXIS_H
#define LLVM_LIB_EXECUTIONENGINE_RUNTIMEDYLD_TARGETS_RUNTIMEDYLDELFMAXIS_H

#include "../RuntimeDyldELF.h"
#include <string>

#define DEBUG_TYPE "dyld"

namespace llvm {

class RuntimeDyldELFMaxis : public RuntimeDyldELF {
public:

  typedef uint64_t TargetPtrT;

  RuntimeDyldELFMaxis(RuntimeDyld::MemoryManager &MM,
                     JITSymbolResolver &Resolver)
      : RuntimeDyldELF(MM, Resolver) {}

  void resolveRelocation(const RelocationEntry &RE, uint64_t Value) override;

protected:
  void resolveMAXISO32Relocation(const SectionEntry &Section, uint64_t Offset,
                                uint32_t Value, uint32_t Type, int32_t Addend);
  void resolveMAXISN32Relocation(const SectionEntry &Section, uint64_t Offset,
                                uint64_t Value, uint32_t Type, int64_t Addend,
                                uint64_t SymOffset, SID SectionID);
  void resolveMAXISN64Relocation(const SectionEntry &Section, uint64_t Offset,
                                uint64_t Value, uint32_t Type, int64_t Addend,
                                uint64_t SymOffset, SID SectionID);

private:
  /// \brief A object file specific relocation resolver
  /// \param RE The relocation to be resolved
  /// \param Value Target symbol address to apply the relocation action
  uint64_t evaluateRelocation(const RelocationEntry &RE, uint64_t Value,
                              uint64_t Addend);

  /// \brief A object file specific relocation resolver
  /// \param RE The relocation to be resolved
  /// \param Value Target symbol address to apply the relocation action
  void applyRelocation(const RelocationEntry &RE, uint64_t Value);

  int64_t evaluateMAXIS32Relocation(const SectionEntry &Section, uint64_t Offset,
                                   uint64_t Value, uint32_t Type);
  int64_t evaluateMAXIS64Relocation(const SectionEntry &Section,
                                   uint64_t Offset, uint64_t Value,
                                   uint32_t Type,  int64_t Addend,
                                   uint64_t SymOffset, SID SectionID);

  void applyMAXISRelocation(uint8_t *TargetPtr, int64_t CalculatedValue,
                           uint32_t Type);

};
}

#undef DEBUG_TYPE

#endif
