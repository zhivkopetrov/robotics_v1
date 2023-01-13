#!/bin/bash

SCRIPT_NAME=`basename "$0"`
echo "Processing $SCRIPT_NAME"

ROS2_DISTRO=foxy BUILD_MOVEIT2=0 docker-compose -f \
    src/ur_driver/Universal_Robots_ROS2_Driver/ur_robot_driver/resources/ursim_driver/docker-compose.yml up