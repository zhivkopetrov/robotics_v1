#!/bin/bash

# install build tools
# default clang/clang++ is 14, which requires gcc/g++-12 libs
apt update && apt-get install -y \
    build-essential \
    clang \
    lld \
    gcc \
    g++

# install utilities
apt update && apt-get install -y \
    cmake \
    git \
    docker \
    docker-compose \
    docker-buildx \
    x11vnc \
    xvfb \
    fluxbox
