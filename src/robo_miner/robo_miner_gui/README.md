# run project from the workspace folder

# setup
. install/setup.bash

# run
ros2 launch robo_miner_gui launch.py

# How to call nested services:
# Arguments must be in a valid YML format.
# Add curcly braces around the nested messages

# API
ros2 service call /query_initial_robot_position robo_miner_interfaces/srv/QueryInitialRobotPosition {}\

ros2 service call /move_robot robo_miner_interfaces/srv/RobotMove "{ robot_move_type: { move_type: 0} }"

ros2 service call /field_map_validate robo_miner_interfaces/srv/FieldMapValidate "{ field_map: {rows: 6, cols: 7, data: [114, 114, 88, 88, 88, 98, 114, 103, 114, 88, 112, 112, 114, 114, 103, 114, 114, 114, 114, 114, 103, 103, 114, 99, 99, 99, 103, 103, 88, 114, 99, 98, 98, 88, 103, 88, 98, 88, 112, 112, 88, 103]} }"

ros2 service call /longest_sequence_validate robo_miner_interfaces/srv/LongestSequenceValidate "{ sequence_points: [ { row: 0, col: 0 }, { row: 0, col: 1 }, { row: 1, col: 1 }, { row: 2, col: 1 }, { row: 3, col: 1 }, { row: 4, col: 1 }, { row: 2, col: 2 }, { row: 2, col: 3 }, { row: 2, col: 4 }, { row: 2, col: 5 }, { row: 1, col: 5 }, { row: 1, col: 6 }, { row: 0, col: 6 } ] }"

ros2 service call /activate_mining_validate robo_miner_interfaces/srv/ActivateMiningValidate {}\

