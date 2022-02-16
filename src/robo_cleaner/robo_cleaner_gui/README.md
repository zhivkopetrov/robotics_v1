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
ros2 service call /robotMove robo_cleaner_interfaces/srv/RobotMove robot_move_type:\ {\ move_type:\ 1}\