//===- MaxisArchTree.cpp --------------------------------------------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===---------------------------------------------------------------------===//
//
// This file contains a helper function for the Writer.
//
//===---------------------------------------------------------------------===//

#include "InputFiles.h"
#include "SymbolTable.h"
#include "Writer.h"

#include "lld/Common/ErrorHandler.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/Object/ELF.h"
#include "llvm/Support/MaxisABIFlags.h"

using namespace llvm;
using namespace llvm::object;
using namespace llvm::ELF;

using namespace lld;
using namespace lld::elf;

namespace {
struct ArchTreeEdge {
  uint32_t Child;
  uint32_t Parent;
};

struct FileFlags {
  InputFile *File;
  uint32_t Flags;
};
} // namespace

static StringRef getAbiName(uint32_t Flags) {
  switch (Flags) {
  case 0:
    return "n64";
  case EF_MAXIS_ABI2:
    return "n32";
  case EF_MAXIS_ABI_O32:
    return "o32";
  case EF_MAXIS_ABI_O64:
    return "o64";
  case EF_MAXIS_ABI_EABI32:
    return "eabi32";
  case EF_MAXIS_ABI_EABI64:
    return "eabi64";
  default:
    return "unknown";
  }
}

static StringRef getNanName(bool IsNan2008) {
  return IsNan2008 ? "2008" : "legacy";
}

static StringRef getFpName(bool IsFp64) { return IsFp64 ? "64" : "32"; }

static void checkFlags(ArrayRef<FileFlags> Files) {
  uint32_t ABI = Files[0].Flags & (EF_MAXIS_ABI | EF_MAXIS_ABI2);
  bool Nan = Files[0].Flags & EF_MAXIS_NAN2008;
  bool Fp = Files[0].Flags & EF_MAXIS_FP64;

  for (const FileFlags &F : Files.slice(1)) {
    uint32_t ABI2 = F.Flags & (EF_MAXIS_ABI | EF_MAXIS_ABI2);
    if (ABI != ABI2)
      error("target ABI '" + getAbiName(ABI) + "' is incompatible with '" +
            getAbiName(ABI2) + "': " + toString(F.File));

    bool Nan2 = F.Flags & EF_MAXIS_NAN2008;
    if (Nan != Nan2)
      error("target -mnan=" + getNanName(Nan) + " is incompatible with -mnan=" +
            getNanName(Nan2) + ": " + toString(F.File));

    bool Fp2 = F.Flags & EF_MAXIS_FP64;
    if (Fp != Fp2)
      error("target -mfp" + getFpName(Fp) + " is incompatible with -mfp" +
            getFpName(Fp2) + ": " + toString(F.File));
  }
}

static uint32_t getMiscFlags(ArrayRef<FileFlags> Files) {
  uint32_t Ret = 0;
  for (const FileFlags &F : Files)
    Ret |= F.Flags &
           (EF_MAXIS_ABI | EF_MAXIS_ABI2 | EF_MAXIS_ARCH_ASE | EF_MAXIS_NOREORDER |
            EF_MAXIS_MICROMAXIS | EF_MAXIS_NAN2008 | EF_MAXIS_32BITMODE);
  return Ret;
}

static uint32_t getPicFlags(ArrayRef<FileFlags> Files) {
  // Check PIC/non-PIC compatibility.
  bool IsPic = Files[0].Flags & (EF_MAXIS_PIC | EF_MAXIS_CPIC);
  for (const FileFlags &F : Files.slice(1)) {
    bool IsPic2 = F.Flags & (EF_MAXIS_PIC | EF_MAXIS_CPIC);
    if (IsPic && !IsPic2)
      warn("linking abicalls code " + toString(Files[0].File) +
           " with non-abicalls file: " + toString(F.File));
    if (!IsPic && IsPic2)
      warn("linking non-abicalls code " + toString(Files[0].File) +
           " with abicalls file: " + toString(F.File));
  }

  // Compute the result PIC/non-PIC flag.
  uint32_t Ret = Files[0].Flags & (EF_MAXIS_PIC | EF_MAXIS_CPIC);
  for (const FileFlags &F : Files.slice(1))
    Ret &= F.Flags & (EF_MAXIS_PIC | EF_MAXIS_CPIC);

  // PIC code is inherently CPIC and may not set CPIC flag explicitly.
  if (Ret & EF_MAXIS_PIC)
    Ret |= EF_MAXIS_CPIC;
  return Ret;
}

static ArchTreeEdge ArchTree[] = {
    // MAXIS32R6 and MAXIS64R6 are not compatible with other extensions
    // MAXIS64R2 extensions.
    {EF_MAXIS_ARCH_64R2 | EF_MAXIS_MACH_OCTEON3, EF_MAXIS_ARCH_64R2},
    {EF_MAXIS_ARCH_64R2 | EF_MAXIS_MACH_OCTEON2, EF_MAXIS_ARCH_64R2},
    {EF_MAXIS_ARCH_64R2 | EF_MAXIS_MACH_OCTEON, EF_MAXIS_ARCH_64R2},
    {EF_MAXIS_ARCH_64R2 | EF_MAXIS_MACH_LS3A, EF_MAXIS_ARCH_64R2},
    // MAXIS64 extensions.
    {EF_MAXIS_ARCH_64 | EF_MAXIS_MACH_SB1, EF_MAXIS_ARCH_64},
    {EF_MAXIS_ARCH_64 | EF_MAXIS_MACH_XLR, EF_MAXIS_ARCH_64},
    {EF_MAXIS_ARCH_64R2, EF_MAXIS_ARCH_64},
    // MAXIS V extensions.
    {EF_MAXIS_ARCH_64, EF_MAXIS_ARCH_5},
    // R5000 extensions.
    {EF_MAXIS_ARCH_4 | EF_MAXIS_MACH_5500, EF_MAXIS_ARCH_4 | EF_MAXIS_MACH_5400},
    // MAXIS IV extensions.
    {EF_MAXIS_ARCH_4 | EF_MAXIS_MACH_5400, EF_MAXIS_ARCH_4},
    {EF_MAXIS_ARCH_4 | EF_MAXIS_MACH_9000, EF_MAXIS_ARCH_4},
    {EF_MAXIS_ARCH_5, EF_MAXIS_ARCH_4},
    // VR4100 extensions.
    {EF_MAXIS_ARCH_3 | EF_MAXIS_MACH_4111, EF_MAXIS_ARCH_3 | EF_MAXIS_MACH_4100},
    {EF_MAXIS_ARCH_3 | EF_MAXIS_MACH_4120, EF_MAXIS_ARCH_3 | EF_MAXIS_MACH_4100},
    // MAXIS III extensions.
    {EF_MAXIS_ARCH_3 | EF_MAXIS_MACH_4010, EF_MAXIS_ARCH_3},
    {EF_MAXIS_ARCH_3 | EF_MAXIS_MACH_4100, EF_MAXIS_ARCH_3},
    {EF_MAXIS_ARCH_3 | EF_MAXIS_MACH_4650, EF_MAXIS_ARCH_3},
    {EF_MAXIS_ARCH_3 | EF_MAXIS_MACH_5900, EF_MAXIS_ARCH_3},
    {EF_MAXIS_ARCH_3 | EF_MAXIS_MACH_LS2E, EF_MAXIS_ARCH_3},
    {EF_MAXIS_ARCH_3 | EF_MAXIS_MACH_LS2F, EF_MAXIS_ARCH_3},
    {EF_MAXIS_ARCH_4, EF_MAXIS_ARCH_3},
    // MAXIS32 extensions.
    {EF_MAXIS_ARCH_32R2, EF_MAXIS_ARCH_32},
    // MAXIS II extensions.
    {EF_MAXIS_ARCH_3, EF_MAXIS_ARCH_2},
    {EF_MAXIS_ARCH_32, EF_MAXIS_ARCH_2},
    // MAXIS I extensions.
    {EF_MAXIS_ARCH_1 | EF_MAXIS_MACH_3900, EF_MAXIS_ARCH_1},
    {EF_MAXIS_ARCH_2, EF_MAXIS_ARCH_1},
};

static bool isArchMatched(uint32_t New, uint32_t Res) {
  if (New == Res)
    return true;
  if (New == EF_MAXIS_ARCH_32 && isArchMatched(EF_MAXIS_ARCH_64, Res))
    return true;
  if (New == EF_MAXIS_ARCH_32R2 && isArchMatched(EF_MAXIS_ARCH_64R2, Res))
    return true;
  for (const auto &Edge : ArchTree) {
    if (Res == Edge.Child) {
      Res = Edge.Parent;
      if (Res == New)
        return true;
    }
  }
  return false;
}

static StringRef getMachName(uint32_t Flags) {
  switch (Flags & EF_MAXIS_MACH) {
  case EF_MAXIS_MACH_NONE:
    return "";
  case EF_MAXIS_MACH_3900:
    return "r3900";
  case EF_MAXIS_MACH_4010:
    return "r4010";
  case EF_MAXIS_MACH_4100:
    return "r4100";
  case EF_MAXIS_MACH_4650:
    return "r4650";
  case EF_MAXIS_MACH_4120:
    return "r4120";
  case EF_MAXIS_MACH_4111:
    return "r4111";
  case EF_MAXIS_MACH_5400:
    return "vr5400";
  case EF_MAXIS_MACH_5900:
    return "vr5900";
  case EF_MAXIS_MACH_5500:
    return "vr5500";
  case EF_MAXIS_MACH_9000:
    return "rm9000";
  case EF_MAXIS_MACH_LS2E:
    return "loongson2e";
  case EF_MAXIS_MACH_LS2F:
    return "loongson2f";
  case EF_MAXIS_MACH_LS3A:
    return "loongson3a";
  case EF_MAXIS_MACH_OCTEON:
    return "octeon";
  case EF_MAXIS_MACH_OCTEON2:
    return "octeon2";
  case EF_MAXIS_MACH_OCTEON3:
    return "octeon3";
  case EF_MAXIS_MACH_SB1:
    return "sb1";
  case EF_MAXIS_MACH_XLR:
    return "xlr";
  default:
    return "unknown machine";
  }
}

static StringRef getArchName(uint32_t Flags) {
  switch (Flags & EF_MAXIS_ARCH) {
  case EF_MAXIS_ARCH_1:
    return "maxis1";
  case EF_MAXIS_ARCH_2:
    return "maxis2";
  case EF_MAXIS_ARCH_3:
    return "maxis3";
  case EF_MAXIS_ARCH_4:
    return "maxis4";
  case EF_MAXIS_ARCH_5:
    return "maxis5";
  case EF_MAXIS_ARCH_32:
    return "maxis32";
  case EF_MAXIS_ARCH_64:
    return "maxis64";
  case EF_MAXIS_ARCH_32R2:
    return "maxis32r2";
  case EF_MAXIS_ARCH_64R2:
    return "maxis64r2";
  case EF_MAXIS_ARCH_32R6:
    return "maxis32r6";
  case EF_MAXIS_ARCH_64R6:
    return "maxis64r6";
  default:
    return "unknown arch";
  }
}

static std::string getFullArchName(uint32_t Flags) {
  StringRef Arch = getArchName(Flags);
  StringRef Mach = getMachName(Flags);
  if (Mach.empty())
    return Arch.str();
  return (Arch + " (" + Mach + ")").str();
}

// There are (arguably too) many MAXIS ISAs out there. Their relationships
// can be represented as a forest. If all input files have ISAs which
// reachable by repeated proceeding from the single child to the parent,
// these input files are compatible. In that case we need to return "highest"
// ISA. If there are incompatible input files, we show an error.
// For example, maxis1 is a "parent" of maxis2 and such files are compatible.
// Output file gets EF_MAXIS_ARCH_2 flag. From the other side maxis3 and maxis32
// are incompatible because nor maxis3 is a parent for misp32, nor maxis32
// is a parent for maxis3.
static uint32_t getArchFlags(ArrayRef<FileFlags> Files) {
  uint32_t Ret = Files[0].Flags & (EF_MAXIS_ARCH | EF_MAXIS_MACH);

  for (const FileFlags &F : Files.slice(1)) {
    uint32_t New = F.Flags & (EF_MAXIS_ARCH | EF_MAXIS_MACH);

    // Check ISA compatibility.
    if (isArchMatched(New, Ret))
      continue;
    if (!isArchMatched(Ret, New)) {
      error("incompatible target ISA:\n>>> " + toString(Files[0].File) + ": " +
            getFullArchName(Ret) + "\n>>> " + toString(F.File) + ": " +
            getFullArchName(New));
      return 0;
    }
    Ret = New;
  }
  return Ret;
}

template <class ELFT> uint32_t elf::calcMaxisEFlags() {
  std::vector<FileFlags> V;
  for (InputFile *F : ObjectFiles)
    V.push_back({F, cast<ObjFile<ELFT>>(F)->getObj().getHeader()->e_flags});
  if (V.empty())
    return 0;
  checkFlags(V);
  return getMiscFlags(V) | getPicFlags(V) | getArchFlags(V);
}

static int compareMaxisFpAbi(uint8_t FpA, uint8_t FpB) {
  if (FpA == FpB)
    return 0;
  if (FpB == Maxis::Val_GNU_MAXIS_ABI_FP_ANY)
    return 1;
  if (FpB == Maxis::Val_GNU_MAXIS_ABI_FP_64A &&
      FpA == Maxis::Val_GNU_MAXIS_ABI_FP_64)
    return 1;
  if (FpB != Maxis::Val_GNU_MAXIS_ABI_FP_XX)
    return -1;
  if (FpA == Maxis::Val_GNU_MAXIS_ABI_FP_DOUBLE ||
      FpA == Maxis::Val_GNU_MAXIS_ABI_FP_64 ||
      FpA == Maxis::Val_GNU_MAXIS_ABI_FP_64A)
    return 1;
  return -1;
}

static StringRef getMaxisFpAbiName(uint8_t FpAbi) {
  switch (FpAbi) {
  case Maxis::Val_GNU_MAXIS_ABI_FP_ANY:
    return "any";
  case Maxis::Val_GNU_MAXIS_ABI_FP_DOUBLE:
    return "-mdouble-float";
  case Maxis::Val_GNU_MAXIS_ABI_FP_SINGLE:
    return "-msingle-float";
  case Maxis::Val_GNU_MAXIS_ABI_FP_SOFT:
    return "-msoft-float";
  case Maxis::Val_GNU_MAXIS_ABI_FP_OLD_64:
    return "-maxis32r2 -mfp64 (old)";
  case Maxis::Val_GNU_MAXIS_ABI_FP_XX:
    return "-mfpxx";
  case Maxis::Val_GNU_MAXIS_ABI_FP_64:
    return "-mgp32 -mfp64";
  case Maxis::Val_GNU_MAXIS_ABI_FP_64A:
    return "-mgp32 -mfp64 -mno-odd-spreg";
  default:
    return "unknown";
  }
}

uint8_t elf::getMaxisFpAbiFlag(uint8_t OldFlag, uint8_t NewFlag,
                              StringRef FileName) {
  if (compareMaxisFpAbi(NewFlag, OldFlag) >= 0)
    return NewFlag;
  if (compareMaxisFpAbi(OldFlag, NewFlag) < 0)
    error("target floating point ABI '" + getMaxisFpAbiName(OldFlag) +
          "' is incompatible with '" + getMaxisFpAbiName(NewFlag) +
          "': " + FileName);
  return OldFlag;
}

template <class ELFT> static bool isN32Abi(const InputFile *F) {
  if (auto *EF = dyn_cast<ELFFileBase<ELFT>>(F))
    return EF->getObj().getHeader()->e_flags & EF_MAXIS_ABI2;
  return false;
}

bool elf::isMaxisN32Abi(const InputFile *F) {
  switch (Config->EKind) {
  case ELF32LEKind:
    return isN32Abi<ELF32LE>(F);
  case ELF32BEKind:
    return isN32Abi<ELF32BE>(F);
  case ELF64LEKind:
    return isN32Abi<ELF64LE>(F);
  case ELF64BEKind:
    return isN32Abi<ELF64BE>(F);
  default:
    llvm_unreachable("unknown Config->EKind");
  }
}

bool elf::isMicroMaxis() { return Config->EFlags & EF_MAXIS_MICROMAXIS; }

bool elf::isMaxisR6() {
  uint32_t Arch = Config->EFlags & EF_MAXIS_ARCH;
  return Arch == EF_MAXIS_ARCH_32R6 || Arch == EF_MAXIS_ARCH_64R6;
}

template uint32_t elf::calcMaxisEFlags<ELF32LE>();
template uint32_t elf::calcMaxisEFlags<ELF32BE>();
template uint32_t elf::calcMaxisEFlags<ELF64LE>();
template uint32_t elf::calcMaxisEFlags<ELF64BE>();
