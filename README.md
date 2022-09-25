# robotics_v1

**A mostly C++ ROS2 workspace**

The workspace contains several interesting visual mini-games with their respective ROS2 interfaces
- Robo Collector - focused on learning ROS2 topics
- Robo Miner - focused on learning ROS2 services
- Robo Cleaner - focused on learning ROS2 actions
- UR Dev - focused on learning UR robotics movements through URScript
- UR Driver - forked repositories of Universal Robots Client Library and Universal Robots ROS2 driver

**Git submodules**
This repository has it's dependencies configured as git submodules.
To clone them, step inside the repository and run the following instructions
```
git submodule init
git submodule update
```

**Ament CMake (Colcon) meta build system usage**
```
colcon build #build all packages with default settings
colcon build --symlink-install #create symbolic links for python, urdf, xacro, yaml files instead of copies
colcon build --packages-up-to <package_name> #build package_name and all of it's dependencies
colcon build --packages-select <package_name> #build only package_name without it's dependencies (faster if you have the dependencies already built)
colcon build --event-handlers console_direct+ --event-handlers console_cohesion+ #show build and install process logs
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER=/usr/bin/clang++ #provide any list of CMake args

#all of the colcon build command can be combined
#for full information check `colcon build --help
```

**Automatic asset information generation**
The first time the project is build it will fail compilation, because you are missing some auto-generated headers.
To resolve this follow the instructios:
```
# build the resource builder tool
colcon build --symlink-install --packages-up-to resource_builder --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER=/usr/bin/clang++

# auto generate C++ headers and asset information for the GUI projects:
./install/resource_builder/lib/resource_builder/resource_builder src/robo_collector/robo_collector_gui
./install/resource_builder/lib/resource_builder/resource_builder src/robo_collector/robo_collector_controller
./install/resource_builder/lib/resource_builder/resource_builder src/robo_miner/robo_miner_gui
./install/resource_builder/lib/resource_builder/resource_builder src/robo_cleaner/robo_cleaner_gui
./install/resource_builder/lib/resource_builder/resource_builder src/ur_dev/ur_control_gui

# build the whole workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER=/usr/bin/clang++
```

**Dependency hierarchy diagram**
![](doc/hierarchy_diagram.svg)


**Previews**
******ur_control_gui + Rviz2 + UR ros driver 2******
![](doc/previews/ur_control_gui.png)

******robo_collector_gui + robo_collector_controller******
![](doc/previews/robo_collector.png)