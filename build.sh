#!/bin/bash
set -ex

: "${BUILD_TYPE:=Debug}"

# TODO(trevor): Move physics.c into it's own build
mkdir -p physics
cp ../../src/physics.c physics/physics_copy.c
#make OPTIONS=USE_GPS TARGET=SITL
docker buildx build --progress=plain --target linux -t betaflight_linux ./docker
docker buildx build --progress=plain --target wasi -t betaflight_wasi ./docker
docker buildx build --progress=plain --target windows -t betaflight_windows ./docker
docker run --rm -it --user 1000:1000 -v `pwd`:/src -e BUILD_TYPE="$BUILD_TYPE" -w /src betaflight_linux ./internal_linux.sh
#docker run --rm -it --user 1000:1000 -v `pwd`:/src -e BUILD_TYPE="$BUILD_TYPE" -w /src betaflight_wasi ./internal_wasi.sh
docker run --rm -it --user 1000:1000 -v `pwd`:/src -e BUILD_TYPE="$BUILD_TYPE" -w /src betaflight_windows ./internal_windows.sh
cp build_linux/libbetaflight_SITL.so ../../Assets/Plugins
cp build_windows/libbetaflight_SITL.dll ../../Assets/Plugins
