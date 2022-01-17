# run project from the workspace folder

colcon build --packages-up-to resource_builder --event-handlers console_direct+ --event-handlers console_cohesion+
. install/setup.bash
ro2 run resource_builder resource_builder src/robo_collector/robo_collector_gui
colcon build --packages-up-to resource_builder --event-handlers console_direct+ --event-handlers console_cohesion+
. install/setup.bash
ros2 run robo_collector_gui robo_collector_gui


# hide the top bar
https://ubuntuhandbook.org/index.php/2020/08/top-panel-auto-hide-ubuntu-20-04/

sudo apt install gnome-shell-extension-autohidetopbar
press ALT + F2, then enter r and hit enter. This will restart the UI
open the extensions app from dash -> hide top bar
open appearance app from dash -> appearance menu -> auto hide dock