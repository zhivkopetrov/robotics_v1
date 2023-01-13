#!/bin/bash

SCRIPT_NAME=`basename "$0"`
echo "Processing $SCRIPT_NAME"
echo "ENABLE_DOCKER_IN_DOCKER: $ENABLE_DOCKER_IN_DOCKER"
echo "ENABLE_VNC_SERVER: $ENABLE_VNC_SERVER"
echo "VNC_DISPLAY: $VNC_DISPLAY"
echo "VNC_PORT: $VNC_PORT"

if [[ "$ENABLE_DOCKER_IN_DOCKER" == "True" ]] ; then
    # start docker daemon
    dockerd &
    sleep 2

    echo "Docker daemon is running"
fi

if [[ "$ENABLE_VNC_SERVER" == "True" ]] ; then
    echo "Starting VNC_SERVER"

    Xvfb ${VNC_DISPLAY} -ac -listen tcp -screen 0 1920x1080x24 &
    sleep 3
    /usr/bin/fluxbox -display ${VNC_DISPLAY} -screen 0 &
    sleep 3
    x11vnc -norc -noxrecord -ncache_cr -forever -shared -bg -loop -rfbport ${VNC_PORT} \
        -display ${VNC_DISPLAY}.0 -passwd ${VNC_PASSWORD:-password} -o /var/log/x11vnc.log &
    sleep 3

    echo "VNC_SERVER is running. You can access it through VNC client"
fi

echo "Configuration completed"
/bin/bash