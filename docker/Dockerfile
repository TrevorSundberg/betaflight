FROM ghcr.io/webassembly/wasi-sdk:sha-53551e5@sha256:8779b83f221769eae6f120b12293b8585875ee50c18af7d93a17c01868c9299c AS wasi

FROM ubuntu:22.04@sha256:d80997daaa3811b175119350d84305e1ec9129e1799bba0bd1e3120da3ff52c3 AS linux

RUN apt-get update && apt-get install -y \
    bash \
    cmake \
    gnupg \
    lsb-release \
    ninja-build \
    software-properties-common \
    wget

RUN wget -qO- https://apt.llvm.org/llvm.sh | bash -s -- 20 && \
    cp /usr/bin/clang-20 /usr/bin/clang && \
    cp /usr/bin/clang-20 /usr/bin/clang++
