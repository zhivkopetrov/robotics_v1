#!/bin/bash

# install build tools
apt update && apt-get install -y \
    build-essential \
    clang \
    gcc \
    g++

# install utilities
apt update && apt-get install -y \
    cmake \
    git \
    docker \
    docker-compose \
    x11vnc \
    xvfb \
    fluxbox