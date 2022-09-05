# run project from the workspace folder

# setup
. install/setup.bash

# run
ros2 launch robo_cleaner_gui launch.py

# How to call nested services:
# Arguments must be in a valid YML format.
# Add curcly braces around the nested messages

# API
ros2 service call /query_initial_robot_state robo_cleaner_interfaces/srv/QueryInitialRobotState {}\

ros2 service call /query_battery_status robo_cleaner_interfaces/srv/QueryBatteryStatus {}\

ros2 action send_goal --feedback /move_robot robo_cleaner_interfaces/action/RobotMove "{ robot_move_type: { move_type: 0 } }"

ros2 service call /charge_battery robo_cleaner_interfaces/srv/ChargeBattery charge_turns:\ 1\