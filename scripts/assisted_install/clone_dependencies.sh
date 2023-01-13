#!/bin/bash

ROS2_DISTRO=foxy

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
    rosdep update && \
    rosdep install --from-paths src -y --ignore-src

# pull docker images from DockerHub containing:
# - Universal Robots ROS2 driver
# - Universal Robots Simulator (URSim) - utilising a ur5 robot
# - Rviz2

docker pull zhivkopetrov90/robotics_accelerator_2022:ursim_img
docker pull zhivkopetrov90/robotics_accelerator_2022:driver_img