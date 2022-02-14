# Author Zhivko Petrov

cmake_minimum_required(VERSION 3.10)

project(
    robo_collector_gui
    LANGUAGES CXX
    VERSION 1.0.0
)

list(
    APPEND
    CMAKE_MODULE_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake_helpers/find_modules
)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake_helpers/helpers.cmake)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robo_collector_interfaces REQUIRED)
find_package(robo_common REQUIRED)
find_package(ros2_game_engine REQUIRED)

set(_INC_FOLDER_NAME include)
set(_RESOURCE_FOLDER_NAME resources)
set(_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${_INC_FOLDER_NAME}/${PROJECT_NAME})
set(_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_executable(
    ${PROJECT_NAME} 
        ${_INC_DIR}/RoboCollectorGui.h
        ${_INC_DIR}/builder/RoboCollectorBuilder.h
        ${_INC_DIR}/config/RoboCollectorGuiConfig.h
        ${_INC_DIR}/config/RoboCollectorGuiConfigGenerator.h
        ${_INC_DIR}/defines/RoboCollectorGuiDefines.h
        ${_INC_DIR}/defines/RoboCollectorGuiFunctionalDefines.h
        ${_INC_DIR}/layout/RoboCollectorLayout.h
        ${_INC_DIR}/layout/RoboCollectorLayoutInterfaces.h
        ${_INC_DIR}/layout/RoboCollectorLayoutInitHelper.h
        ${_INC_DIR}/layout/config/RoboCollectorLayoutConfig.h
        ${_INC_DIR}/entities/robot/RobotAI.h
        ${_INC_DIR}/entities/coin/Coin.h
        ${_INC_DIR}/entities/coin/CoinHandler.h
        ${_INC_DIR}/entities/coin/config/CoinHandlerConfig.h
        ${_INC_DIR}/entities/coin/animation/CoinCollectAnimEndCb.h
        ${_INC_DIR}/entities/coin/animation/CoinRespawnAnim.h
        ${_INC_DIR}/panels/PanelHandler.h
        ${_INC_DIR}/panels/CoinPanel.h
        ${_INC_DIR}/panels/config/CoinPanelConfig.h
        ${_INC_DIR}/panels/config/PanelHandlerConfig.h
        ${_INC_DIR}/controller/buttons/MoveButton.h
        ${_INC_DIR}/controller/buttons/HelpButton.h
        ${_INC_DIR}/controller/buttons/SettingsButton.h
        ${_INC_DIR}/controller/RoboCollectorController.h
        ${_INC_DIR}/controller/config/RoboCollectorControllerConfig.h
        ${_INC_DIR}/helpers/TurnHelper.h
        ${_INC_DIR}/helpers/RoboCollectorGuiInitHelper.h
        ${_INC_DIR}/external_api/ControllerExternalBridge.h
        
        ${_INC_DIR}/robo_miner/RoboMinerGui.h
        ${_INC_DIR}/robo_miner/defines/RoboMinerDefines.h
        ${_INC_DIR}/robo_miner/config/RoboMinerGuiConfig.h
        ${_INC_DIR}/robo_miner/entities/Crystal.h
        ${_INC_DIR}/robo_miner/field/RoboMinerField.h
        ${_INC_DIR}/robo_miner/field/config/RoboMinerFieldConfig.h
        ${_INC_DIR}/robo_miner/algorithms/FloodFill.h
        
        ${_INC_DIR}/robo_cleaner/RoboCleanerGui.h
        ${_INC_DIR}/robo_cleaner/config/RoboCleanerGuiConfig.h
        ${_INC_DIR}/robo_cleaner/panels/config/EnergyPanelConfig.h
        ${_INC_DIR}/robo_cleaner/panels/EnergyPanel.h
        ${_INC_DIR}/robo_cleaner/entities/Rubbish.h
     
        ${_SRC_DIR}/main.cpp
        ${_SRC_DIR}/RoboCollectorGui.cpp
        ${_SRC_DIR}/builder/RoboCollectorBuilder.cpp
        ${_SRC_DIR}/config/RoboCollectorGuiConfigGenerator.cpp
        ${_SRC_DIR}/layout/RoboCollectorLayout.cpp
        ${_SRC_DIR}/layout/RoboCollectorLayoutInitHelper.cpp
        ${_SRC_DIR}/entities/robot/RobotAI.cpp
        ${_SRC_DIR}/entities/coin/Coin.cpp
        ${_SRC_DIR}/entities/coin/CoinHandler.cpp
        ${_SRC_DIR}/entities/coin/animation/CoinCollectAnimEndCb.cpp
        ${_SRC_DIR}/entities/coin/animation/CoinRespawnAnim.cpp
        ${_SRC_DIR}/panels/PanelHandler.cpp
        ${_SRC_DIR}/panels/CoinPanel.cpp
        ${_SRC_DIR}/controller/buttons/MoveButton.cpp
        ${_SRC_DIR}/controller/buttons/HelpButton.cpp
        ${_SRC_DIR}/controller/buttons/SettingsButton.cpp
        ${_SRC_DIR}/controller/RoboCollectorController.cpp
        ${_SRC_DIR}/helpers/TurnHelper.cpp
        ${_SRC_DIR}/helpers/RoboCollectorGuiInitHelper.cpp
        ${_SRC_DIR}/external_api/ControllerExternalBridge.cpp
        
        ${_SRC_DIR}/robo_miner/RoboMinerGui.cpp
        ${_SRC_DIR}/robo_miner/defines/RoboMinerDefines.cpp
        ${_SRC_DIR}/robo_miner/entities/Crystal.cpp
        ${_SRC_DIR}/robo_miner/field/RoboMinerField.cpp
        ${_SRC_DIR}/robo_miner/algorithms/FloodFill.cpp
        
        ${_SRC_DIR}/robo_cleaner/RoboCleanerGui.cpp
        ${_SRC_DIR}/robo_cleaner/panels/EnergyPanel.cpp
        ${_SRC_DIR}/robo_cleaner/entities/Rubbish.cpp
)

target_include_directories(
    ${PROJECT_NAME} 
    PRIVATE
        $<BUILD_INTERFACE:
            ${CMAKE_CURRENT_SOURCE_DIR}/${_INC_FOLDER_NAME}
            ${CMAKE_CURRENT_SOURCE_DIR}/${_RESOURCE_FOLDER_NAME}
        >
        $<INSTALL_INTERFACE:
            ${_INC_FOLDER_NAME}
            ${_RESOURCE_FOLDER_NAME}
        >
)

target_link_libraries(
    ${PROJECT_NAME}
        robo_common::robo_common
        ros2_game_engine::ros2_game_engine
)

ament_target_dependencies(
    ${PROJECT_NAME}
        robo_collector_interfaces
        rclcpp
        ament_index_cpp
)

set_target_cpp_standard(${PROJECT_NAME} 20)
enable_target_warnings(${PROJECT_NAME})
# enable_target_sanitizer(${PROJECT_NAME} "memory")

if(NOT DISABLE_ROS_TOOLING)
    enable_ros_tooling_for_target(
        ${PROJECT_NAME}
        ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
    )
endif()

# Install resources and auto-generated headers and resource bin filles
install(
    DIRECTORY ${_RESOURCE_FOLDER_NAME}/
    DESTINATION share/${PROJECT_NAME}/${_RESOURCE_FOLDER_NAME}/
)

