# run project from the workspace folder

# setup
. install/setup.bash

# run
ros2 launch robo_collector_gui launch.py

# API
ros2 topic pub --once /move_type robo_collector_interfaces/msg/RobotMoveType move_type:\ 0\
