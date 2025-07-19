#!/bin/bash
set -ex

rm -rf build_windows
mkdir -p build_windows

cmake \
  -S . -B build_windows \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_SYSTEM_NAME=Windows \
  -DCMAKE_C_COMPILER=/usr/bin/clang \
  -DCMAKE_CXX_COMPILER=/usr/bin/clang++ \
  -DCMAKE_C_COMPILER_TARGET=x86_64-w64-windows-gnu \
  -DCMAKE_CXX_COMPILER_TARGET=x86_64-w64-windows-gnu \
  -DCMAKE_RC_COMPILER=x86_64-w64-mingw32-windres

#cmake \
#  -S . -B build_windows \
#  -G Ninja \
#  -DCMAKE_BUILD_TYPE=Debug \
#  -DCMAKE_SYSTEM_NAME=Windows \
#  -DCMAKE_C_COMPILER=/usr/bin/clang \
#  -DCMAKE_CXX_COMPILER=/usr/bin/clang++ \
#  -DCMAKE_C_COMPILER_TARGET=x86_64-w64-mingw32 \
#  -DCMAKE_CXX_COMPILER_TARGET=x86_64-w64-mingw32 \
#  -DCMAKE_RC_COMPILER=x86_64-w64-mingw32-windres

cmake --build build_windows --parallel
