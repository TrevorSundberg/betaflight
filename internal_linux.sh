#!/bin/bash
set -ex

rm -rf build_linux
mkdir -p build_linux

cmake \
  -S . -B build_linux \
  -G Ninja \
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
  -DCMAKE_C_COMPILER=/usr/bin/clang \
  -DCMAKE_CXX_COMPILER=/usr/bin/clang++

cmake --build build_linux --parallel
