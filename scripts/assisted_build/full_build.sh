#!/bin/bash

SCRIPT_NAME=`basename "$0"`
BUILD_TYPE=Debug
INSTALL_DIR=install
VERBOSE_BUILD=False
ADDITIONAL_COLCON_OPTIONS=""

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

#build auto generated asset info
bash ./scripts/assisted_build/generate_asset_info.sh \
    $BUILD_TYPE $VERBOSE_BUILD "$ADDITIONAL_COLCON_OPTIONS" 2>&1

#build and install artifacts
bash ./scripts/assisted_build/partial_build.sh \
    $BUILD_TYPE $VERBOSE_BUILD "$ADDITIONAL_COLCON_OPTIONS" 2>&1
 