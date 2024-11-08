# robotics_v1

## An educational C++20 Robot Operating System 2 (ROS2) Jazzy Jalisco workspace
This is the official repository for the Robotics Accelerator course, created by me and powered by Ocado Technology.
  - More on the training - [Robotics Accelerator page](https://pages.beamery.eu/ocadogroup/page/ot-sofia-roboticscourse2022) / [Official Video](https://www.youtube.com/watch?v=wgNpRyYdaUs)

The project utilizes a personal 2D game_engine set of libraries and highly configurable thread-per-component module  architecture.
  - More on the game_engine - refer to its [documentation](https://github.com/zhivkopetrov/game_engine)

The workspace contains several interesting, competitive, visual games with their respective ROS2 interfaces
- [Robo Collector](https://github.com/zhivkopetrov/robotics_v1/tree/master/src/robo_collector) - focused on learning ROS2 topics
- [Robo Miner](https://github.com/zhivkopetrov/robotics_v1/tree/master/src/robo_miner) - focused on learning ROS2 services
- [Robo Cleaner](https://github.com/zhivkopetrov/robotics_v1/tree/master/src/robo_cleaner) - focused on learning ROS2 actions
- [UR Dev](https://github.com/zhivkopetrov/robotics_v1/tree/master/src/ur_dev) - focused on learning UR robotic motions through URScript

*You're reading the documentation for the latest version of ROS2 - Jazzy Jalisco.  
An older, but still supported, version of ROS 2 - Humble Hawksbill, available under the [humble](https://github.com/zhivkopetrov/robotics_v1/tree/humble) branch.  
An older, but End-of-Life ROS2 version - Foxy Fitzroy is still supported by this repo, available under the [foxy](https://github.com/zhivkopetrov/robotics_v1/tree/foxy) branch.*

## Build Status
<table width="100%">
  <tr>
    <th>ROS2 Distro</th>
    <th>Jazzy Jalisco</th>
    <th>Humble Hawksbill</th>
    <th>Foxy Fitzroy</th>
  </tr>
  <tr>
    <th>Branch</th>
    <td><div align="center"><a href="https://github.com/zhivkopetrov/robotics_v1/tree/master">master</a></div></td>
    <td><div align="center"><a href="https://github.com/zhivkopetrov/robotics_v1/tree/humble">humble</a></div></td>
    <td><div align="center"><a href="https://github.com/zhivkopetrov/robotics_v1/tree/foxy">foxy</a></div></td>
  </tr>
  <tr>
    <th>Build Status</th>
      <th><a href="https://github.com/zhivkopetrov/robotics_v1/actions/workflows/jazzy_docker_image.yml">
         <img src="https://github.com/zhivkopetrov/robotics_v1/actions/workflows/jazzy_docker_image.yml/badge.svg"
              alt="Jazzy Build"/></a></th>
      <th><a href="https://github.com/zhivkopetrov/robotics_v1/actions/workflows/humble_docker_image.yml">
         <img src="https://github.com/zhivkopetrov/robotics_v1/actions/workflows/humble_docker_image.yml/badge.svg"
              alt="Humble Build"/></a></th>
      <th><a href="https://github.com/zhivkopetrov/robotics_v1/actions/workflows/foxy_docker_image.yml">
         <img src="https://github.com/zhivkopetrov/robotics_v1/actions/workflows/foxy_docker_image.yml/badge.svg"
              alt="Foxy Build"/></a></th>
  </tr>
  <tr>
    <th>OS version</th>
    <th>Ubuntu 24.04 LTS</th>
    <th>Ubuntu 22.04 LTS</th>
    <th>Ubuntu 20.04 LTS</th>
  </tr>
  <tr>
    <th>Documentation</th>
    <td><div align="center"><a href="https://docs.ros.org/en/jazzy/index.html">ROS2 Jazzy Jalisco</a></div></td>
    <td><div align="center"><a href="https://docs.ros.org/en/humble/index.html">ROS2 Humble Hawksbill</a></div></td>
    <td><div align="center"><a href="https://docs.ros.org/en/foxy/index.html">ROS2 Foxy Fitzroy</a></div></td>
  </tr>
</table>

## Previews
### ur_control_gui + Rviz2 + UR ros driver 2
![](doc/previews/ur_control_gui.png)

### robo_collector_gui + robo_collector_controller
![](doc/previews/robo_collector.png)

## Supported Platforms & Compilers
- Linux
  - g++ (9.3 and above)
    - Tested up to g++ 12.1
  - clang++ (10 and above)
    - Tested up to clang++ 14.0

- Windows
  - MSVC++ (14.20 and above) Visual Studio 2019
    - Tested up to 17.30 Visual Studio 2022
    - Note: enable Linux Bash Shell support under Windows to utilise the preset build scripts
    - Note2: although the game-engine is fully MSVC++ compatible, I haven't tested actual ROS2 functionalities on Windows

## Project automated installation 
All dependencies in the project could be conveniently installed via preset install scripts. 
For manual installation refer to 'Dependencies' and 'Third party libs' sections below.

Please note that ROS2 installation could be quite bulky.
For reference, a fresh Ubuntu 22.04 docker image with all compilers, tools, libs and ROS2 installed is close to 7GB.
That number could be reduced, but it's not a focus for this repository.
### Host usage
```
# Warning, the script will install dependencies directly on your host
sudo ./scripts/assisted_install/full_install_on_host.sh
```
Once the installation phase finishes, for convinience, you can add manual steps to your bashrc.
This way each time you open a NEW terminal session, they will be populated for you.
```
echo "export LANG=en_US.UTF-8" >> ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source source /usr/share/colcon_cd/function/colcon_cd-argcomplete.bash" >> ~/.bashrc
```
### Docker support
  - Requires Docker Engine (18.09.1 and above)
    - Tested up to Docker Engine 20.10.23
```
# All parameters are optional

# Clone dependencies on fresh Ubuntu 24.04 image, build and install artifacts
./scripts/assisted_install/full_install_in_docker.sh <os_version> <ros2_distro> <enable_vnc_server> <enable_docker_in_docker>

# By default os_version=ubuntu:24.04 ros2_distro=jazzy, enable_vnc_server=False, enable_docker_in_docker=False

# Start the image
./scripts/run/run_docker_file.sh <ros2_distro> <enable_privileged_mode>

# By default ros2_distro=jazzy, enable_privileged_mode=False
# Privileged mode is required if you want to enable docker in docker support.
# For example starting the Universal Robots Simulator docker image inside robotics_v1 docker image (Docker in Docker)

# To access the VNC Server use a VNC Client of your choice
# For example VNC Viewer - https://www.realvnc.com/en/connect/download/viewer/

Access the VNC Server using local host and port 5920
Address: 127.0.0.1:5920
Password: robotics_v1

Doing so will lead you to Fluxbox.
Fluxbox is an extemely basic and lightweight stacking window manager for the X Window System.
All you need is probably a terminal session.
Right click -> Applications -> Shells -> Bash
```

## Colcon configuration. Building the project
Use plain Colcon commands to configure and build the project or use some of the existing preset build scripts.
### Basic usage
Automatic asset generation + build + install steps
```
./scripts/assisted_build/full_build.sh
```
Build + install steps
```
./scripts/assisted_build/partial_build.sh
```
### Advanced usage
```
# all parameters listed in the scripts are optional

# A full build is required only once in the beginning.
# For more information refer to the 'Automatic asset information generation' section
./scripts/assisted_build/full_build.sh <build_type> <verbose_build> <additional_colcon_options>

# A partial build is the prefered, fast, with minimal compilation (build + install) procedure
./scripts/assisted_build/partial_build.sh <build_type> <verbose_build> <additional_colcon_options>

# <build_type> - Debug / Release / RelWithDebInfo / MinSizeRel
# Defaults to Debug
# This param is passed to CMAKE_BUILD_TYPE
# https://cmake.org/cmake/help/latest/variable/CMAKE_BUILD_TYPE.html

# <verbose_build> - True / False
# Defaults to False
# Verbose output for build and install steps

# <additional_colcon_options> - colcon specific parameters
# Defaults to none

# For plain colcon commands run 'colcon build --help'
# Or refer to the official colcon documentation:
# https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html
```

## Automatic asset information generation
The first time the project is manually build it will fail compilation.
The reason is because of missing auto-generated headers.
Those headers contain asset information such as image/sprite/text/sound texture data, asset locations, load policies, etc...
To auto-generate them use the following instructions:
```
# all parameters listed in the scripts are optional

./scripts/assisted_build/generate_asset_info.sh <build_type> <verbose_build> <additional_colcon_options>

# Note: the build preset script 'full_build.sh' already invoke 'generate_asset_info.sh' for you
```
Asset information generation should be executed only once in the beginning.
Or everytime you modify some of the assets or resource files (asset information descriptions)
  - More on resource files - check the [resource_builder tool documentation](https://github.com/zhivkopetrov/tools)

## Running the project
The project is composed of multiple ros2 packages, most of which contain a GUI.  
**Important:** as per colcon documentation - don't run ROS2 packages from the same terminal session used for building.  
Instead run them from a separate terminal session.

### General guidance
Once the project is build - source its installed artifacts.  
This should be done for each NEW terminal session
```
source install/setup.bash
```
Run nodes via normal colcon commands (ros2 run/launch).  

### Robo games
```
ros2 launch robo_collector_gui launch.py
ros2 launch robo_collector_controller launch.py
ros2 launch robo_miner_gui launch.py
ros2 launch robo_cleaner_gui launch.py
# Note that 'ros2 launch' will load config files, while 'ros2 run' will use the default ones
```

### Universal Robots Application suite setup
If real hardware robot is used, the application suite will pick it up automatically.  
If instead a Universal Robots Simulator (URSim) is used, start it with:  
```
./scripts/run/run_ursim.sh <ur_type>
```
Supported ur_types: ur3/ur3e/ur5/ur5e/ur10/ur10e.  
ur16/ur16e are supported as parameters, but their URDFs are not present.  
Note that URSim does not support ARM based CPUs.  

Starting the Universal Robots ROS2 driver:
```
./scripts/run/run_ur_driver.sh <ur_type> <robot_ip> <launch_rviz>
```
Example usage with ur10e robot configured to connect with URSim
```
./scripts/run/run_ur_driver.sh ur10e 192.168.56.101 true
```

Helper utility node, exposing beginner-friendly API from the robot.  
```
ros2 launch urscript_bridge launch.py
```

Helper GUI nodes, which can control real hardware or a simulated robot.  
The nodes are utilising the robot API exposed from the 'urscript_bridge' node.  
```
ros2 launch ur_control_gui launch.py
ros2 launch ur_control_bloom launch.py
```

`ur_control_gui` is a high-level control application suited for executing raw URScripts.  
`ur_control_bloom` provides more sophisticated control concepts such as:
  - Execution of blocking or async motion commands
  - Graceful/immediate abort of motion trajectories
  - Constructing gripper/motion/robot URScripts with source code primitives
  - Configurable high-level state machine and low-level motion sequences
  - Configurable motion strategies (behavior) based on config 
  - Gripper full hardware support and basic simulation support

## Hardware requirements
This project utilizes hardware-accelerated graphics.
Any low-end GPU (integrated or dedicated) will be sufficient.
CPU usage is neglectable for the majority of projects and low-demanding for the Universal Robots Driver.

If however hardware-accelerated graphics could not be used - the GUI fallbacks to software rendering.
This could be the case if the project is ran inside a Virtual Machine with no hardware-acceleration.
Or if a cloud-hosted Virtual Machine is used. The majority of those lack hardware-acceleration support.
If Software rendering is used - the CPU usage will skyrocket.
By default the GUI is confgured to run at 60 FPS, which when using software rendering may hit the 100% CPU mark.

## Support for hardware-restrained (CPU cores and/or RAM) - a.k.a. toaster build
By default the colcon build system utilizes a parallel executor (simultaneous package build/install instructions).
If your package dependency graph allows it up-to <hardware_concurrency> CPU cores could be utilised.
```
# The simultaneous processed packages could be controlled with <additional_colcon_options>
--parallel-workers NUMBER (defaults to <hardware_concurrency>)
```

Additionally, the generation of ROS2 interfaces is an especially RAM intensive operation.
- Reference stats for a clean project build on 8 parallel CPU cores:
  - Using clang++ yields a peak of 800% CPU usage and 5 GB of RAM usage
  - Using g++ yields a peak of 800% CPU usage and 9 GB of RAM usage
    - Any subsequent builds then utilize only a small portion of the hardware resources 

To relief the hardware usage spike for initial builds:
```
# Pass a ' --parallel-workers 1' as <additional_colcon_options>

# Or completely change the executor type to 'sequential'
# Pass a ' --executor sequential' as <additional_colcon_options>
# You can achieve this step conveniently by invoking the following preset build script:
./scripts/assisted_build/toaster_build.sh <build_type> <verbose_build> <additional_colcon_options>

# any parameters passed to <additional_colcon_options> will be appended to the ' --executor sequential' option
```

## Dependencies (personal open-source libraries and tools)
This step decribes the manual installation of dependencies. For automated/assisted install refer to the 'Project automated installation' section.
Dependencies are integrated as git submodules. 
- To clone them, step inside the repository and run the following instructions
```
git submodule init
git submodule update
```
Those commands will clone the following repositories:
- [cmake_helpers](https://github.com/zhivkopetrov/cmake_helpers.git)
- [utils](https://github.com/zhivkopetrov/utils)
- [resource_utils](https://github.com/zhivkopetrov/resource_utils)
- [sdl_utils](https://github.com/zhivkopetrov/sdl_utils)
- [manager_utils](https://github.com/zhivkopetrov/manager_utils)
- [game_engine](https://github.com/zhivkopetrov/game_engine)
- [ros2_game_engine](https://github.com/zhivkopetrov/ros2_game_engine)
- [tools](https://github.com/zhivkopetrov/tools)

## Third party libs, which are not shipped with this repository
This step decribes the manual installation of third party libs. For automated/assisted install refer to the 'Project automated installation' section.
- ROS2 Jazzy Jalisco
  - For installation, please refer to the official [ROS2 Jazzy Jalisco installation documentation](https://docs.ros.org/en/jazzy/Installation.html)

- Boost
  - Linux
    - Install through apt is sufficient
      ```
      sudo apt install libboost-dev
      ```
  - Windows
    - Refer to the official [boost installation documentation](https://www.boost.org/doc/libs/1_60_0/more/getting_started/windows.html#install-boost-build)

- SDL2 family libraries - SDL2, SDL2-image, SDL2-ttf, SDL2-mixer
  - Linux
    - Install through apt is sufficient
      ```
      sudo apt install libsdl2-dev libsdl2-ttf-dev libsdl2-image-dev libsdl2-mixer-dev
      ```
  - Windows
    - Download SDL2 packages from the official [SDL2 repository](https://github.com/libsdl-org/SDL/releases/latest)
    - Extract under "C:/SDL2" or alongside the project binary 

## Dependency hierarchy diagrams
### Robo Games
![](doc/hierarchy_diagrams/robo_games_hierarchy_diagram.svg)

### UR Dev
![](doc/hierarchy_diagrams/ur_dev_hierarchy_diagram.svg)
