#!/bin/bash
SCRIPT_NAME=`basename "$0"`
ROBOT_TYPE=ur10e
ROBOT_IP=192.168.56.101
LAUNCH_RVIZ=true

echo "Processing $SCRIPT_NAME"

if [ -z "$1" ]; then
    echo "No argument supplied for ROBOT_TYPE. Assuming '$ROBOT_TYPE'"
else
    ROBOT_TYPE=$1
    echo "Using ROBOT_TYPE=$ROBOT_TYPE"
fi

if [ -z "$2" ]; then
    echo "No argument supplied for ROBOT_IP. Assuming '$ROBOT_IP'"
else
    ROBOT_IP=$2
    echo "Using ROBOT_IP=$ROBOT_IP"
fi

if [ -z "$3" ]; then
    echo "No argument supplied for LAUNCH_RVIZ. Assuming '$LAUNCH_RVIZ'"
else
    LAUNCH_RVIZ=$3
    if [[ "$LAUNCH_RVIZ" == "true" || 
          "$LAUNCH_RVIZ" == "True" || 
          "$LAUNCH_RVIZ" == "1" ]] ; then
        LAUNCH_RVIZ=true
    else
        LAUNCH_RVIZ=false
    fi

    echo "Using LAUNCH_RVIZ=$LAUNCH_RVIZ"
fi

source install/setup.bash

ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=$ROBOT_TYPE \
    robot_ip:=$ROBOT_IP \
    launch_rviz:=$LAUNCH_RVIZ