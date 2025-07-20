#!/bin/bash
set -ex

rm -rf build_wasi
mkdir -p build_wasi

cmake \
  -S . -B build_wasi \
  -G Ninja \
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
  -DCMAKE_TOOLCHAIN_FILE=/opt/wasi-sdk/share/cmake/wasi-sdk-p2.cmake

cmake --build build_wasi --parallel
