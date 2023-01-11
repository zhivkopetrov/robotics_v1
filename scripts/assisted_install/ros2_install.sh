#!/bin/bash

ROS2_DISTRO=humble

if [ -z "$1" ]; then
    echo "No argument supplied for ROS2_DISTRO. Assuming '$ROS2_DISTRO'"
else
    ROS2_DISTRO=$1
    echo "Using ROS2_DISTRO=$ROS2_DISTRO"
fi

apt update && apt install locales -y
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

apt install software-properties-common -y
add-apt-repository universes

apt update && apt install curl -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt update && apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools

apt update && apt install -y \
  python3-flake8-blind-except \
  python3-flake8-builtins \
  python3-flake8-class-newline \
  python3-flake8-comprehensions \
  python3-flake8-deprecated \
  python3-flake8-import-order \
  python3-flake8-quotes \
  python3-pytest-repeat \
  python3-pytest-rerunfailures \
  python3-colcon-common-extensions \
  python3-vcstool

apt update && apt install -y \
  ros-$ROS2_DISTRO-desktop-full \
  ros-$ROS2_DISTRO-ros2cli \
  ros-$ROS2_DISTRO-ros2controlcli \
  ros-$ROS2_DISTRO-ros2cli-common-extensions \
  ros-$ROS2_DISTRO-ros2bag \
  ros-$ROS2_DISTRO-joint-trajectory-controller \
  ros-$ROS2_DISTRO-ur-msgs
