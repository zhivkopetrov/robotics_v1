#!/bin/bash

# Script parameters
ROS2_DISTRO=humble
ENABLE_VNC_SERVER=False
ENABLE_DOCKER_IN_DOCKER=False

# Environment variables
CMAKE_BUILD_TYPE="$CMAKE_BUILD_TYPE:-Debug"
VERBOSE_BUILD="$VERBOSE_BUILD:-False"

if [ -z "$1" ]; then
    echo "No argument supplied for ROS2_DISTRO. Assuming '$ROS2_DISTRO'"
else
    ROS2_DISTRO=$1
    echo "Using ROS2_DISTRO=$ROS2_DISTRO"
fi

if [ -z "$2" ]; then
    echo "No argument supplied for ENABLE_VNC_SERVER. Assuming '$ENABLE_VNC_SERVER'"
else
    ENABLE_VNC_SERVER=$2
    if [[ "$ENABLE_VNC_SERVER" == "true" || 
          "$ENABLE_VNC_SERVER" == "True" || 
          "$ENABLE_VNC_SERVER" == "1" ]] ; then
        ENABLE_VNC_SERVER=True
    else
        ENABLE_VNC_SERVER=False
    fi

    echo "Using ENABLE_VNC_SERVER=$ENABLE_VNC_SERVER"
fi

if [ -z "$3" ]; then
    echo "No argument supplied for ENABLE_DOCKER_IN_DOCKER. Assuming '$ENABLE_DOCKER_IN_DOCKER'"
else
    ENABLE_DOCKER_IN_DOCKER=$3
    if [[ "$ENABLE_DOCKER_IN_DOCKER" == "true" || 
          "$ENABLE_DOCKER_IN_DOCKER" == "True" || 
          "$ENABLE_DOCKER_IN_DOCKER" == "1" ]] ; then
        ENABLE_DOCKER_IN_DOCKER=True
    else
        ENABLE_DOCKER_IN_DOCKER=False
    fi

    echo "Using ENABLE_DOCKER_IN_DOCKER=$ENABLE_DOCKER_IN_DOCKER"
fi

DOCKER_BUILDKIT=1 docker build \
    --file docker/Dockerfile \
    --build-arg ROS2_DISTRO=$ROS2_DISTRO \
    --build-arg ENABLE_VNC_SERVER=$ENABLE_VNC_SERVER \
    --build-arg ENABLE_DOCKER_IN_DOCKER=$ENABLE_DOCKER_IN_DOCKER \
    --build-arg CMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE \
    --build-arg VERBOSE_BUILD=$VERBOSE_BUILD \
    --build-arg ADDITIONAL_COLCON_OPTIONS=$ADDITIONAL_COLCON_OPTIONS \
    --tag robotics_v1:$ROS2_DISTRO .