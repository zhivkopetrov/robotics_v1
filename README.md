# robotics_v1

**A mostly C++ ROS2 workspace**

The workspace contains several intetesting visual mini-games and their respective ROS2 interfaces
- Robo Collector - focused on learning ROS2 topics
- Robo Miner - focused on learning ROS2 services
- Robo Cleaner - focused on learning ROS2 actions

```plantuml
@startuml
package "static C++ libs" {
  [cmake_helpers]
  [utils]
  [resource_utils]
  [sdl_utils]
  [manager_utils]
  [game_engine]
  [ros2_game_engine]
}

package "tools" {
  [resource_builder]
}

package "auto-generated" {
  [cpp_headers]
  [asset_information]
}

[robo_common]

package "robo_collector" {
  [robo_collector_gui]
  [robo_collector_controller]
  [robo_collector_common]
  [robo_collector_interfaces]
}

package "robo_miner" {
  [robo_miner_gui]
  [robo_miner_common]
  [robo_miner_interfaces]
}

package "robo_cleaner" {
  [robo_cleaner_gui]
  [robo_cleaner_common]
  [robo_cleaner_interfaces]
}

' START libs & tools
cmake_helpers -> utils
utils -> resource_utils
resource_utils --> sdl_utils
sdl_utils -> manager_utils
manager_utils -> game_engine
game_engine -> ros2_game_engine

resource_utils --> resource_builder
resource_builder --> asset_information
resource_builder --> cpp_headers
asset_information --> game_engine
cpp_headers --> game_engine
' END libs & tools

manager_utils --> robo_common

' START Robo Collector
robo_collector_interfaces --> robo_collector_common
robo_common --> robo_collector_common
robo_collector_common --> robo_collector_gui
robo_collector_common --> robo_collector_controller
ros2_game_engine --> robo_collector_gui
ros2_game_engine --> robo_collector_controller
' END Robo Collector

' START Robo Miner
robo_miner_interfaces --> robo_miner_common
robo_common --> robo_miner_common
robo_miner_common --> robo_miner_gui
ros2_game_engine --> robo_miner_gui
' END Robo Miner

' START Robo Cleaner
robo_cleaner_interfaces --> robo_cleaner_common
robo_common --> robo_cleaner_common
robo_cleaner_common --> robo_cleaner_gui
ros2_game_engine --> robo_cleaner_gui
' END Robo Cleaner

@enduml
```