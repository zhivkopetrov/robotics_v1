FROM ubuntu:22.04 as base

ARG ROS2_DISTRO=humble
ARG REPO_NAME=robotics_v1
ARG INSTALL_SCRIPTS_DIR=/scripts/assisted_install
ARG BUILD_SCRIPTS_DIR=/scripts/assisted_build

# Noninteractive option for time-zone settings
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# clone main repository
RUN apt update && apt install git -y
RUN git clone https://github.com/zhivkopetrov/${REPO_NAME}.git
WORKDIR ${REPO_NAME}

# Split into multiple steps to use docker cache layering

# compilers and tools
RUN .${INSTALL_SCRIPTS_DIR}/ubuntu_apt_base_install.sh

# third party libs
RUN .${INSTALL_SCRIPTS_DIR}/ubuntu_apt_third_party_install.sh

# complere ROS2 setup per documentation
# https://docs.ros.org/en/humble/Installation.html
RUN .${INSTALL_SCRIPTS_DIR}/ros2_install.sh $ROS2_DISTRO

# own dependencies (personal open-source libraries and tools)
RUN .${INSTALL_SCRIPTS_DIR}/clone_dependencies.sh $ROS2_DISTRO

# build and install artifacts
RUN .${BUILD_SCRIPTS_DIR}/full_build.sh

# start container with
# docker run --privileged --rm -it --entrypoint bash robotics_v1:humble

# start docker daemon inside docker
# dockerd &