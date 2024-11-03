#!/bin/bash

# install general utilities
apt update && apt-get install -y \
    curl \
    git

# install docker
apt update && apt-get install -y \
    docker-compose \
    docker-buildx

# install build tools
apt update && apt-get install -y \
    build-essential \
    clang \
    lld \
    gcc \
    g++ \
    cmake

# install VNC related utilities
apt update && apt-get install -y \
    x11vnc \
    xvfb \
    fluxbox