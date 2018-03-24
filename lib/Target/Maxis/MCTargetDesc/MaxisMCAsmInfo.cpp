//===-- MaxisMCAsmInfo.cpp - Maxis Asm Properties ---------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the declarations of the MaxisMCAsmInfo properties.
//
//===----------------------------------------------------------------------===//

#include "MaxisMCAsmInfo.h"
#include "llvm/ADT/Triple.h"

using namespace llvm;

void MaxisMCAsmInfo::anchor() { }

MaxisMCAsmInfo::MaxisMCAsmInfo(const Triple &TheTriple) {
  IsLittleEndian = TheTriple.isLittleEndian();

  if ((TheTriple.getArch() == Triple::maxis64el) ||
      (TheTriple.getArch() == Triple::maxis64)) {
    CodePointerSize = CalleeSaveStackSlotSize = 8;
  }

  // FIXME: This condition isn't quite right but it's the best we can do until
  //        this object can identify the ABI. It will misbehave when using O32
  //        on a maxis64*-* triple.
  if ((TheTriple.getArch() == Triple::maxisel) ||
      (TheTriple.getArch() == Triple::maxis)) {
    PrivateGlobalPrefix = "$";
    PrivateLabelPrefix = "$";
  }

  AlignmentIsInBytes          = false;
  Data16bitsDirective         = "\t.2byte\t";
  Data32bitsDirective         = "\t.4byte\t";
  Data64bitsDirective         = "\t.8byte\t";
  CommentString               = "#";
  ZeroDirective               = "\t.space\t";
  GPRel32Directive            = "\t.gpword\t";
  GPRel64Directive            = "\t.gpdword\t";
  DTPRel32Directive           = "\t.dtprelword\t";
  DTPRel64Directive           = "\t.dtpreldword\t";
  TPRel32Directive            = "\t.tprelword\t";
  TPRel64Directive            = "\t.tpreldword\t";
  UseAssignmentForEHBegin = true;
  SupportsDebugInformation = true;
  ExceptionsType = ExceptionHandling::DwarfCFI;
  DwarfRegNumForCFI = true;
  HasMaxisExpressions = true;

  // Enable IAS by default for O32.
  if (TheTriple.getArch() == Triple::maxis ||
      TheTriple.getArch() == Triple::maxisel)
    UseIntegratedAssembler = true;

  // Enable IAS by default for Debian maxis64/maxis64el.
  if (TheTriple.getEnvironment() == Triple::GNUABI64)
    UseIntegratedAssembler = true;

  // Enable IAS by default for Android maxis64el that uses N64 ABI.
  if (TheTriple.getArch() == Triple::maxis64el && TheTriple.isAndroid())
    UseIntegratedAssembler = true;
}
