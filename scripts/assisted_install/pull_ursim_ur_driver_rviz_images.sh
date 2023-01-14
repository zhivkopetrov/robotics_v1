#!/bin/bash

# pull docker images from DockerHub containing:
# - Universal Robots ROS2 driver
# - Universal Robots Simulator (URSim) - utilising a ur5 robot
# - Rviz2
docker pull zhivkopetrov90/robotics_accelerator_2022:ursim_img
docker pull zhivkopetrov90/robotics_accelerator_2022:driver_img