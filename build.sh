#!/bin/sh
rm -rf build
mkdir build
cd build

cmake \
    -DCMAKE_C_COMPILER=/usr/local/bin/gcc -DCMAKE_CXX_COMPILER=/usr/local/bin/g++ \
    -DCMAKE_BUILD_TYPE="DEBUG" \
    -DCMAKE_C_FLAGS="-g -ggdb" \
    -DCMAKE_CXX_FLAGS="-g -ggdb" \
    -DLLVM_TARGETS_TO_BUILD="Maxis" \
    ..


#    -DCMAKE_BUILD_TYPE="RelWithDebInfo" \
#    -DLLVM_OPTIMIZED_TABLEGEN=ON \

#    -DLLVM_TARGETS_TO_BUILD="Maxis" \
#    -DLLVM_TARGETS_TO_BUILD="Axis" \
#    -DLLVM_TARGETS_TO_BUILD="Axis;Mips;X86" \
#    -DCMAKE_C_COMPILER=/usr/bin/clang \
#    -DCMAKE_CXX_COMPILER=/usr/bin/clang++ \

#    -DLLVM_BUILD_TOOLS=OFF \
#    -DLLVM_APPEND_VC_REV=OFF \
#    -DLLVM_INCLUDE_TOOLS=OFF \

make -j`grep -c processor /proc/cpuinfo` 

