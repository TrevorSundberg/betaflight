#!/bin/bash
set -ex

rm -rf build_wasi
mkdir -p build_wasi
cd build_wasi

cmake -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=/opt/wasi-sdk/share/cmake/wasi-sdk-p2.cmake ..
cmake --build . --parallel
