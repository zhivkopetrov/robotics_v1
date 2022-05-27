# run project from the workspace folder

colcon build --packages-up-to resource_builder --event-handlers console_direct+ --event-handlers console_cohesion+
. install/setup.bash
ros2 run resource_builder resource_builder src/robo_miner/robo_miner_gui
#if ROS2 is unable to find the package run the command from the install folder:
./install/resource_builder/lib/resource_builder/resource_builder src/robo_miner/robo_miner_gui
colcon build --packages-up-to resource_builder --event-handlers console_direct+ --event-handlers console_cohesion+
. install/setup.bash
ros2 run robo_miner_gui robo_miner_gui


# how to call nested services
# arguments must be in a valid YML format
# add curcly braces around the nested messages
ros2 service call /move_robot robo_miner_interfaces/srv/RobotMove "{ robot_move_type: { move_type: 0} }"

ros2 service call /field_map_validate robo_miner_interfaces/srv/FieldMapValidate "{ field_map: {rows: 6, cols: 7, data: [114, 114, 88, 88, 88, 98, 114, 103, 114, 88, 112, 112, 114, 114, 103, 114, 114, 114, 114, 114, 103, 103, 114, 99, 99, 99, 103, 103, 88, 114, 99, 98, 98, 88, 103, 88, 98, 88, 112, 112, 88, 103]} }"

ros2 service call /longest_sequence_validate robo_miner_interfaces/srv/LongestSequenceValidate "{ sequence_points: [ { row: 0, col: 0 }, { row: 0, col: 1 }, { row: 1, col: 1 }, { row: 2, col: 1 }, { row: 3, col: 1 }, { row: 4, col: 1 }, { row: 2, col: 2 }, { row: 2, col: 3 }, { row: 2, col: 4 }, { row: 2, col: 5 }, { row: 1, col: 5 }, { row: 1, col: 6 }, { row: 0, col: 6 } ] }"

ros2 service call /activate_mining_validate robo_miner_interfaces/srv/ActivateMiningValidate {}\

