FROM ubuntu:20.04 as base

ARG ROS2_DISTRO=foxy
ARG REPO_NAME=robotics_v1
ARG INSTALL_SCRIPTS_DIR=/scripts/assisted_install
ARG BUILD_SCRIPTS_DIR=/scripts/assisted_build

ARG ENABLE_DOCKER_IN_DOCKER=False
ENV ENABLE_DOCKER_IN_DOCKER=${ENABLE_DOCKER_IN_DOCKER}

# Noninteractive option for time-zone settings
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# VNC variables
ARG ENABLE_VNC_SERVER=False
ENV ENABLE_VNC_SERVER=${ENABLE_VNC_SERVER}
ARG VNC_DISPLAY=:20
ENV VNC_DISPLAY=${VNC_DISPLAY}
ARG VNC_PORT=5920
ENV VNC_PORT=${VNC_PORT}
ARG VNC_PASSWORD="${REPO_NAME}"
ENV VNC_PASSWORD=${VNC_PASSWORD}

# clone main repository
RUN apt update && apt install git -y
RUN git clone https://github.com/zhivkopetrov/${REPO_NAME}.git
WORKDIR ${REPO_NAME}

# Split into multiple steps to use docker cache layering

# Compilers and tools
RUN .${INSTALL_SCRIPTS_DIR}/ubuntu_apt_base_install.sh

# Third party libs
RUN .${INSTALL_SCRIPTS_DIR}/ubuntu_apt_third_party_install.sh

# Complete ROS2 setup per documentation
# https://docs.ros.org/en/foxy/Installation.html
RUN .${INSTALL_SCRIPTS_DIR}/ros2_install.sh ${ROS2_DISTRO}

# Own dependencies (personal open-source libraries and tools)
RUN .${INSTALL_SCRIPTS_DIR}/clone_dependencies.sh ${ROS2_DISTRO}

# Installation clean Up
RUN rm -rf \
    /tmp/* \
    /var/lib/apt/lists/* \
    /var/tmp/*

# build and install artifacts
RUN .${BUILD_SCRIPTS_DIR}/full_build.sh

# Expose port 5920 to view display using VNC
EXPOSE ${VNC_PORT}

# Container entrypoint
COPY --chmod=755 /docker/entrypoint.sh .
ENTRYPOINT ["./entrypoint.sh"]