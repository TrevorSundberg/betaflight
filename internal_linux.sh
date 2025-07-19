#!/bin/bash
set -ex

rm -rf build_linux
mkdir -p build_linux
cd build_linux

cmake -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_C_COMPILER=/usr/bin/clang -DCMAKE_CXX_COMPILER=/usr/bin/clang++ ..
cmake --build . --parallel
