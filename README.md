# robotics_v1

## A C++20 ROS2 Humble Hawksbill workspace
This is the official repository for the Robotics Accelerator course, created by me and powered by Ocado Technology.
  - More on the training - https://pages.beamery.eu/ocadogroup/page/ot-sofia-roboticscourse2022

The project utilizes a personal 2D game_engine set of libraries and highly configurable thread-per-component module  architecture.
  - More on the game_engine - refer to its documentation https://github.com/zhivkopetrov/game_engine

The workspace contains several interesting, competitive, visual games with their respective ROS2 interfaces
- Robo Collector - focused on learning ROS2 topics
- Robo Miner - focused on learning ROS2 services
- Robo Cleaner - focused on learning ROS2 actions
- UR Dev - focused on learning UR robotics movements through URScript
- UR Driver - forked helper repositories for Universal Robots Client Library and Universal Robots ROS2 driver + description

## Previews
### ur_control_gui + Rviz2 + UR ros driver 2
![](doc/previews/ur_control_gui.png)

### robo_collector_gui + robo_collector_controller
![](doc/previews/robo_collector.png)

## ROS2 disribution
This repository operates under ROS2 Humble Hawksbill disribution.
  - This is the official ROS2 disribution for Ubuntu 22.04 LTS.
    - https://docs.ros.org/en/humble/index.html
  
ROS2 Foxy Fitzroy implementation is still be accessible under the 'foxy' branch
  - ROS2 Foxy Fitzroy is the oficial ROS2 disribution for Ubuntu 20.04 LTS
    - https://docs.ros.org/en/foxy/index.html

## Supported Platforms & Compilers
- Linux
  - g++ 12
  - clang++ 14

- Windows
  - MSVC++ (>= 14.20) Visual Studio 2019
    - Note: enable Linux Bash Shell support under Windows to utilise the preset build scripts
    - Note2: although the game-engine is fully MSVC++ compatible, I haven't tested actual ROS2 functionalities on Windows

## Colcon configuration. Building the project
Use plain Colcon commands to configure and build the project or use some of the existing preset build scripts
### Basic usage
```
./scripts/assisted_build/full_build.sh
```
```
./scripts/assisted_build/partial_build.sh
```
### Advanced usage
```
# all parameters listed in the scripts are optional

# full build - automatic asset generation + build + install steps
# A full build is required only once in the beginning.
# For more information refer to the 'Automatic asset information generation' section
./scripts/assisted_build/full_build.sh <build_type> <verbose_build> <additional_colcon_options>

# partial build - build + install steps
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
# Or refer to the official documentation:
# https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html
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
  - More on resource files - check the resource_builder tool documentation:
    - https://github.com/zhivkopetrov/tools

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
Dependencies are integrated as git submodules. 
- To clone them, step inside the repository and run the following instructions
```
git submodule init
git submodule update
```
Those commands will clone the following repositories:
- cmake_helpers - https://github.com/zhivkopetrov/cmake_helpers.git
- utils - https://github.com/zhivkopetrov/utils
- resource_utils - https://github.com/zhivkopetrov/resource_utils
- sdl_utils - https://github.com/zhivkopetrov/sdl_utils
- manager_utils - https://github.com/zhivkopetrov/manager_utils
- game_engine - https://github.com/zhivkopetrov/game_engine
- ros2_game_engine - https://github.com/zhivkopetrov/ros2_game_engine
- tools - https://github.com/zhivkopetrov/tools

## Third party libs, which are not shipped with this repository
- ROS2 Humble Hawksbill
  - For installation, please refer to the official documentation
    - https://docs.ros.org/en/humble/Installation.html

- SDL2 family libraries
  - SDL2
  - SDL2-image
  - SDL2-ttf
  - SDL2-mixer

Installing SDL2 family libraries through apt is sufficient when Linux native builds are targeted 
```
sudo apt install libsdl2-dev libsdl2-ttf-dev libsdl2-image-dev libsdl2-mixer-dev
```

For windows builds download SDL2 packages from the official repo
https://github.com/libsdl-org/SDL/releases/latest

Extract under "C:/SDL2" or alongside the project binary 

## Dependency hierarchy diagram
![](doc/hierarchy_diagram.svg)
