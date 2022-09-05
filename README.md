# robotics_v1

**A mostly C++ ROS2 workspace**

The workspace contains several intetesting visual mini-games and their respective ROS2 interfaces
- Robo Collector - focused on learning ROS2 topics
- Robo Miner - focused on learning ROS2 services
- Robo Cleaner - focused on learning ROS2 actions

**Ament CMake (Colcon) meta build system usage**
```
colcon build #build all packages with default settings
colcon build --symlink-install #create symbolic links for python, urdf, xacro, yaml files instead of copies
colcon build --packages-up-to <package_name> #build package_name and all of it's dependencies
colcon build --packages-select <package_name> #build only package_name without it's dependencies (faster if you have the dependencies already built)
colcon build --event-handlers console_direct+ --event-handlers console_cohesion+ #show build and install process logs
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER=/usr/bin/clang++ #provide any list of CMake args

#all of the colconn build command can be combined
#for full information check `colcon build --help
```

**Dependency hierarchy diagram**
![](doc/hierarchy_diagram.svg)