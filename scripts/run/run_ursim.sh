#!/bin/bash
SCRIPT_NAME=`basename "$0"`
ROBOT_TYPE=ur10e

echo "Processing $SCRIPT_NAME"

if [ -z "$1" ]; then
    echo "No argument supplied for ROBOT_TYPE. Assuming '$ROBOT_TYPE'"
else
    ROBOT_TYPE=$1
    echo "Using ROBOT_TYPE=$ROBOT_TYPE"
fi

source install/setup.bash

bash ./scripts/run/start_ursim.sh -m $ROBOT_TYPE