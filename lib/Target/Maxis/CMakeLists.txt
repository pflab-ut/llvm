set(LLVM_TARGET_DEFINITIONS Maxis.td)

tablegen(LLVM MaxisGenRegisterInfo.inc -gen-register-info)
tablegen(LLVM MaxisGenInstrInfo.inc -gen-instr-info)
tablegen(LLVM MaxisGenDisassemblerTables.inc -gen-disassembler)
tablegen(LLVM MaxisGenMCCodeEmitter.inc -gen-emitter)
tablegen(LLVM MaxisGenAsmWriter.inc -gen-asm-writer)
tablegen(LLVM MaxisGenDAGISel.inc -gen-dag-isel)
tablegen(LLVM MaxisGenFastISel.inc -gen-fast-isel)
tablegen(LLVM MaxisGenCallingConv.inc -gen-callingconv)
tablegen(LLVM MaxisGenSubtargetInfo.inc -gen-subtarget)
tablegen(LLVM MaxisGenAsmMatcher.inc -gen-asm-matcher)
tablegen(LLVM MaxisGenMCPseudoLowering.inc -gen-pseudo-lowering)
add_public_tablegen_target(MaxisCommonTableGen)

add_llvm_target(MaxisCodeGen
  Maxis16FrameLowering.cpp
  Maxis16HardFloat.cpp
  Maxis16HardFloatInfo.cpp
  Maxis16InstrInfo.cpp
  Maxis16ISelDAGToDAG.cpp
  Maxis16ISelLowering.cpp
  Maxis16RegisterInfo.cpp
  MaxisAnalyzeImmediate.cpp
  MaxisAsmPrinter.cpp
  MaxisCCState.cpp
  MaxisConstantIslandPass.cpp
  MaxisDelaySlotFiller.cpp
  MaxisFastISel.cpp
  MaxisHazardSchedule.cpp
  MaxisInstrInfo.cpp
  MaxisISelDAGToDAG.cpp
  MaxisISelLowering.cpp
  MaxisFrameLowering.cpp
  MaxisLongBranch.cpp
  MaxisMCInstLower.cpp
  MaxisMachineFunction.cpp
  MaxisModuleISelDAGToDAG.cpp
  MaxisOptimizePICCall.cpp
  MaxisOs16.cpp
  MaxisRegisterInfo.cpp
  MaxisSEFrameLowering.cpp
  MaxisSEInstrInfo.cpp
  MaxisSEISelDAGToDAG.cpp
  MaxisSEISelLowering.cpp
  MaxisSERegisterInfo.cpp
  MaxisSubtarget.cpp
  MaxisTargetMachine.cpp
  MaxisTargetObjectFile.cpp
  MicroMaxisSizeReduction.cpp
  )

add_subdirectory(InstPrinter)
add_subdirectory(Disassembler)
add_subdirectory(TargetInfo)
add_subdirectory(MCTargetDesc)
add_subdirectory(AsmParser)

