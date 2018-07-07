#!/bin/sh
rm -rf build
mkdir build
cd build

cmake \
    -DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
    -DCMAKE_BUILD_TYPE="DEBUG" \
    -DCMAKE_C_FLAGS="-g -ggdb" \
    -DCMAKE_CXX_FLAGS="-g -ggdb" \
    -DLLVM_TARGETS_TO_BUILD="Maxis" \
    ..


# build compiler
#    -DCMAKE_C_COMPILER=/usr/bin/clang \
#    -DCMAKE_CXX_COMPILER=/usr/bin/clang++ \

# build type
#    -DCMAKE_BUILD_TYPE="RelWithDebInfo" \

# build target (if all, do not set LLVM_TARGETS_TO_BUILD)
#    -DLLVM_TARGETS_TO_BUILD="Maxis" \
#    -DLLVM_TARGETS_TO_BUILD="Axis" \
#    -DLLVM_TARGETS_TO_BUILD="Axis;Mips;X86" \


make -j`grep -c processor /proc/cpuinfo` 

