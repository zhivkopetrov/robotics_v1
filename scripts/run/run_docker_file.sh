#!/bin/bash
SCRIPT_NAME=`basename "$0"`
REPO_NAME=robotics_v1
ROS2_DISTRO=humble
PRIVILEGED_MODE=False
PRIVILEGED_STR=""

echo "Processing $SCRIPT_NAME"

if [ -z "$1" ]; then
    echo "No argument supplied for ROS2_DISTRO. Assuming '$ROS2_DISTRO'"
else
    ROS2_DISTRO=$1
    echo "Using ROS2_DISTRO=$ROS2_DISTRO"
fi

if [ -z "$2" ]; then
    echo "No argument supplied for PRIVILEGED_MODE. Assuming '$PRIVILEGED_MODE'"
else
    PRIVILEGED_MODE=$2
    if [[ "$PRIVILEGED_MODE" == "true" || 
          "$PRIVILEGED_MODE" == "True" || 
          "$PRIVILEGED_MODE" == "1" ]] ; then
        PRIVILEGED_STR+="--privileged"
    fi

    echo "Using PRIVILEGED_MODE=$PRIVILEGED_MODE"
fi

docker run -it --rm -p 5920:5920 $PRIVILEGED_STR $REPO_NAME:$ROS2_DISTRO