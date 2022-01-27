# Author Zhivko Petrov

cmake_minimum_required(VERSION 3.10)

project(
    robo_collector_gui
    LANGUAGES CXX
    VERSION 1.0.0
)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_helpers/find_modules)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake_helpers/helpers.cmake)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robo_collector_interfaces REQUIRED)
find_package(game_engine REQUIRED)

set(_INC_FOLDER_NAME include)
set(_RESOURCE_FOLDER_NAME resources)
set(_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${_INC_FOLDER_NAME}/${PROJECT_NAME})
set(_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_executable(
    ${PROJECT_NAME} 
        ${_INC_DIR}/RoboCollectorGui.h
        ${_INC_DIR}/config/RoboCollectorGuiConfig.h
        ${_INC_DIR}/config/RoboCollectorGuiConfigGenerator.h
        ${_INC_DIR}/defines/RoboCollectorGuiDefines.h
        ${_INC_DIR}/defines/RoboCollectorGuiFunctionalDefines.h
        ${_INC_DIR}/field/Field.h
        ${_INC_DIR}/field/FieldPos.h
        ${_INC_DIR}/field/FieldUtils.h
        ${_INC_DIR}/field/Tile.h
        ${_INC_DIR}/field/config/FieldConfig.h
        ${_INC_DIR}/field/config/TileConfig.h
        ${_INC_DIR}/entities/robot/Robot.h
        ${_INC_DIR}/entities/robot/RobotAI.h
        ${_INC_DIR}/entities/robot/RobotUtils.h
        ${_INC_DIR}/entities/robot/config/RobotBaseConfig.h
        ${_INC_DIR}/entities/robot/animation/RobotAnimEndCb.h
        ${_INC_DIR}/entities/coin/Coin.h
        ${_INC_DIR}/entities/coin/CoinHandler.h
        ${_INC_DIR}/entities/coin/animation/CoinCollectAnimEndCb.h
        ${_INC_DIR}/entities/coin/animation/CoinRespawnAnim.h
        ${_INC_DIR}/panels/PanelHandler.h
        ${_INC_DIR}/panels/TimePanel.h
        ${_INC_DIR}/panels/CoinPanel.h
        ${_INC_DIR}/panels/HealthPanel.h
        ${_INC_DIR}/panels/config/HealthPanelConfig.h
        ${_INC_DIR}/panels/config/CoinPanelConfig.h
        ${_INC_DIR}/panels/config/TimePanelConfig.h
        ${_INC_DIR}/panels/config/PanelHandlerConfig.h
        ${_INC_DIR}/controller/MoveButton.h
        ${_INC_DIR}/controller/RoboCollectorController.h
        ${_INC_DIR}/helpers/CollisionObject.h
        ${_INC_DIR}/helpers/CollisionWatcher.h
        ${_INC_DIR}/helpers/TurnHelper.h
     
        ${_SRC_DIR}/main.cpp
        ${_SRC_DIR}/RoboCollectorGui.cpp
        ${_SRC_DIR}/config/RoboCollectorGuiConfigGenerator.cpp
        ${_SRC_DIR}/field/Field.cpp
        ${_SRC_DIR}/field/FieldPos.cpp
        ${_SRC_DIR}/field/FieldUtils.cpp
        ${_SRC_DIR}/field/Tile.cpp
        ${_SRC_DIR}/entities/robot/Robot.cpp
        ${_SRC_DIR}/entities/robot/RobotAI.cpp
        ${_SRC_DIR}/entities/robot/RobotUtils.cpp
        ${_SRC_DIR}/entities/robot/animation/RobotAnimEndCb.cpp
        ${_SRC_DIR}/entities/coin/Coin.cpp
        ${_SRC_DIR}/entities/coin/CoinHandler.cpp
        ${_SRC_DIR}/entities/coin/animation/CoinCollectAnimEndCb.cpp
        ${_SRC_DIR}/entities/coin/animation/CoinRespawnAnim.cpp
        ${_SRC_DIR}/panels/PanelHandler.cpp
        ${_SRC_DIR}/panels/TimePanel.cpp
        ${_SRC_DIR}/panels/CoinPanel.cpp
        ${_SRC_DIR}/panels/HealthPanel.cpp
        ${_SRC_DIR}/controller/MoveButton.cpp
        ${_SRC_DIR}/controller/RoboCollectorController.cpp
        ${_SRC_DIR}/helpers/CollisionWatcher.cpp
        ${_SRC_DIR}/helpers/TurnHelper.cpp
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
        game_engine::game_engine
)

ament_target_dependencies(
    ${PROJECT_NAME}
        robo_collector_interfaces
        rclcpp
        ament_index_cpp
)

set_target_cpp_standard(${PROJECT_NAME} 20)
enable_target_warnings(${PROJECT_NAME})
#set_target_visibility(${PROJECT_NAME})

if(NOT DISABLE_ROS_TOOLING)
    enable_ros_tooling_for_target(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/package.xml)
endif()

# Install resources and auto-generated headers and resource bin filles
install(
    DIRECTORY ${_RESOURCE_FOLDER_NAME}/
    DESTINATION share/${PROJECT_NAME}/${_RESOURCE_FOLDER_NAME}/
)

