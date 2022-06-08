# run project from the workspace folder

colcon build --packages-up-to resource_builder --event-handlers console_direct+ --event-handlers console_cohesion+
. install/setup.bash
ros2 run resource_builder resource_builder src/robo_cleaner/robo_cleaner_gui
#if ROS2 is unable to find the package run the command from the install folder:
./install/resource_builder/lib/resource_builder/resource_builder src/robo_cleaner/robo_cleaner_gui
colcon build --packages-up-to resource_builder --event-handlers console_direct+ --event-handlers console_cohesion+
. install/setup.bash
ros2 run robo_cleaner_gui robo_cleaner_gui


# how to call nested services
# arguments must be in a valid YML format
# add curcly braces around the nested messages

ros2 service call /query_initial_robot_state robo_cleaner_interfaces/srv/QueryInitialRobotState {}\

ros2 service call /query_battery_status robo_cleaner_interfaces/srv/QueryBatteryStatus {}\

ros2 action send_goal --feedback /move_robot robo_cleaner_interfaces/action/RobotMove "{ robot_move_type: { move_type: 0 } }"