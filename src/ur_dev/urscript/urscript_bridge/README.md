# run project from the workspace folder

. install/setup.bash
ros2 launch urscript_bridge launch.py

OR

ros2 run urscript_bridge urscript_bridge
#note: ros2 run will not load parameters listed in the config/params.yaml
