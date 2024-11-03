#!/bin/bash

ROS2_DISTRO=humble

if [ -z "$1" ]; then
    echo "No argument supplied for ROS2_DISTRO. Assuming '$ROS2_DISTRO'"
else
    ROS2_DISTRO=$1
    echo "Using ROS2_DISTRO=$ROS2_DISTRO"
fi

# clone personal open-source repositories
git submodule init && git submodule update

source /opt/ros/$ROS2_DISTRO/setup.bash

# install dependencies from source
rosdep init && \
    rosdep fix-permissions && \
    rosdep update && \
    rosdep install --from-paths src -y --ignore-src