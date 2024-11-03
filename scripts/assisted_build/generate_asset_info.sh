#!/bin/bash

SCRIPT_NAME=`basename "$0"`
BUILD_TYPE=Debug
SRC_DIR=src
INSTALL_DIR=install
VERBOSE_BUILD=False
ADDITIONAL_COLCON_OPTIONS=""
TOOL_NAME=resource_builder
ROS2_DISTRO=jazzy

echo "Processing $SCRIPT_NAME"

if [ -z "$1" ]; then
    echo "No argument supplied for BUILD_TYPE. Assuming '$BUILD_TYPE'"
else
    BUILD_TYPE=$1
    echo "Using BUILD_TYPE=$BUILD_TYPE"
fi

if [ -z "$2" ]; then
    echo "No argument supplied for VERBOSE_BUILD. Assuming '$VERBOSE_BUILD'"
else
    VERBOSE_BUILD=$2
    if [[ "$VERBOSE_BUILD" == "true" || 
          "$VERBOSE_BUILD" == "True" || 
          "$VERBOSE_BUILD" == "1" ]] ; then
        VERBOSE_BUILD=True
        ADDITIONAL_COLCON_OPTIONS+=" --event-handlers console_direct+"
        ADDITIONAL_COLCON_OPTIONS+=" --event-handlers console_cohesion+"
    else
        VERBOSE_BUILD=False
    fi

    echo "Using VERBOSE_BUILD=$VERBOSE_BUILD"
fi

if [ -z "$3" ]; then
    echo "No ADDITIONAL_COLCON_OPTIONS provided"
else
    ADDITIONAL_COLCON_OPTIONS+="$3"
    echo "Using ADDITIONAL_COLCON_OPTIONS=$ADDITIONAL_COLCON_OPTIONS"
fi

source /opt/ros/$ROS2_DISTRO/setup.bash

colcon build \
    --symlink-install \
    --packages-up-to $TOOL_NAME \
    $ADDITIONAL_COLCON_OPTIONS \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
        -DCMAKE_CXX_COMPILER=/usr/bin/clang++ \
        -DCMAKE_LINKER=lld \
        -DCMAKE_EXE_LINKER_FLAGS="-fuse-ld=lld" \
        -DCMAKE_SHARED_LINKER_FLAGS="-fuse-ld=lld" \
        -DCMAKE_MODULE_LINKER_FLAGS="-fuse-ld=lld" 2>&1

GUI_PROJECTS=(\
    "$SRC_DIR/robo_collector/robo_collector_gui" \
    "$SRC_DIR/robo_collector/robo_collector_controller" \
    "$SRC_DIR/robo_miner/robo_miner_gui" \
    "$SRC_DIR/robo_cleaner/robo_cleaner_gui" \
    "$SRC_DIR/ur_dev/ur_control/ur_control_gui" \
    "$SRC_DIR/ur_dev/ur_control/ur_control_bloom"
)

for project in ${GUI_PROJECTS[@]}; do
    $INSTALL_DIR/$TOOL_NAME/lib/$TOOL_NAME/$TOOL_NAME $project/ 2>&1
done
