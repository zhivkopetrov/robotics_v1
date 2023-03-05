#!/bin/bash

# install build tools
# default clang/clang++ is 14, which requires gcc/g++-12 libs
apt update && apt-get install -y \
    build-essential \
    clang

# install utilities
apt update && apt-get install -y \
    cmake \
    git \
    docker \
    docker-compose \
    x11vnc \
    xvfb \
    fluxbox
