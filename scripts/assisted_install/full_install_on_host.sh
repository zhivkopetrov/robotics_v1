#!/bin/bash

ROS2_DISTRO=foxy

# Noninteractive option for time-zone settings
export DEBIAN_FRONTEND=noninteractive
export TZ=Etc/UTC

# compilers and tools
./scripts/assisted_install/ubuntu_apt_base_install.sh

# third party libs
./scripts/assisted_install/ubuntu_apt_third_party_install.sh

# complete ROS2 setup per documentation
# https://docs.ros.org/en/foxy/Installation.html
./scripts/assisted_install/ros2_install.sh $ROS2_DISTRO

# own dependencies (personal open-source libraries and tools)
./scripts/assisted_install/clone_dependencies.sh $ROS2_DISTRO
