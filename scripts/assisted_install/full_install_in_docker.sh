#!/bin/bash

ROS2_DISTRO=humble

if [ -z "$1" ]; then
    echo "No argument supplied for ROS2_DISTRO. Assuming '$ROS2_DISTRO'"
else
    ROS2_DISTRO=$1
    echo "Using ROS2_DISTRO=$ROS2_DISTRO"
fi

DOCKER_BUILDKIT=1 docker build --file docker/Dockerfile --tag robotics_v1:$ROS2_DISTRO --no-cache .