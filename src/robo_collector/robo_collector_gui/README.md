# run project from the workspace folder

colcon build --packages-up-to resource_builder --event-handlers console_direct+ --event-handlers console_cohesion+
. install/setup.bash
ro2 run resource_builder resource_builder src/robo_collector/robo_collector_gui
colcon build --packages-up-to resource_builder --event-handlers console_direct+ --event-handlers console_cohesion+
. install/setup.bash
ros2 run robo_collector_gui robo_collector_gui