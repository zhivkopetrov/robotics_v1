FROM ubuntu:22.04 as base

ARG ROS2_DISTRO=humble
ARG REPO_NAME=robotics_v1

# Noninteractive option for time-zone settings
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Split into multiple steps to use docker cache layering
COPY --chmod=0755 /scripts/assisted_install/ubuntu_apt_base_install.sh .
RUN ./ubuntu_apt_base_install.sh

COPY --chmod=0755 /scripts/assisted_install/ubuntu_apt_third_party_install.sh .
RUN ./ubuntu_apt_third_party_install.sh

COPY --chmod=0755 /scripts/assisted_install/ros2_install.sh .
RUN ./ros2_install.sh ${ROS2_DISTRO}

# clone main repository
RUN git clone https://github.com/zhivkopetrov/${REPO_NAME}.git

WORKDIR ${REPO_NAME}

COPY --chmod=0755 /scripts/assisted_install/clone_dependencies.sh .
RUN ./clone_dependencies.sh ${ROS2_DISTRO}

RUN ./scripts/assisted_build/full_build.sh

# start container with
# docker run --privileged --rm -it --entrypoint bash robotics_v1:humble

# start docker daemon inside docker
# dockerd &