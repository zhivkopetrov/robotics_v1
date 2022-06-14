# Author Zhivko Petrov

cmake_minimum_required(VERSION 3.10)

project(
    robo_collector_gui
    LANGUAGES CXX
    VERSION 1.0.0
)

find_package(cmake_helpers REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robo_collector_interfaces REQUIRED)
find_package(robo_collector_common REQUIRED)
find_package(ros2_game_engine REQUIRED)

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
        ${_INC_DIR}/helpers/RoboCollectorBuilder.h
        ${_INC_DIR}/helpers/TurnHelper.h
        ${_INC_DIR}/helpers/RoboCollectorGuiInitHelper.h
        ${_INC_DIR}/external_api/CollectorControllerExternalBridge.h
        ${_INC_DIR}/external_api/RoboCollectorGuiRos2ParamProvider.h
        ${_INC_DIR}/layout/RoboCollectorLayout.h
        ${_INC_DIR}/layout/helpers/RoboCollectorLayoutInterfaces.h
        ${_INC_DIR}/layout/helpers/RoboCollectorLayoutInitHelper.h
        ${_INC_DIR}/layout/config/RoboCollectorLayoutConfig.h
        ${_INC_DIR}/layout/entities/robot/RobotAI.h
        ${_INC_DIR}/layout/entities/coin/Coin.h
        ${_INC_DIR}/layout/entities/coin/CoinHandler.h
        ${_INC_DIR}/layout/entities/coin/config/CoinHandlerConfig.h
        ${_INC_DIR}/layout/entities/coin/animation/CoinCollectAnimEndCb.h
        ${_INC_DIR}/layout/entities/coin/animation/CoinRespawnAnim.h
        ${_INC_DIR}/layout/panels/PanelHandler.h
        ${_INC_DIR}/layout/panels/config/PanelHandlerConfig.h
     
        ${_SRC_DIR}/main.cpp
        ${_SRC_DIR}/RoboCollectorGui.cpp
        ${_SRC_DIR}/config/RoboCollectorGuiConfigGenerator.cpp
        ${_SRC_DIR}/helpers/TurnHelper.cpp
        ${_SRC_DIR}/helpers/RoboCollectorBuilder.cpp
        ${_SRC_DIR}/helpers/RoboCollectorGuiInitHelper.cpp
        ${_SRC_DIR}/external_api/CollectorControllerExternalBridge.cpp
        ${_SRC_DIR}/external_api/RoboCollectorGuiRos2ParamProvider.cpp
        ${_SRC_DIR}/layout/RoboCollectorLayout.cpp
        ${_SRC_DIR}/layout/helpers/RoboCollectorLayoutInitHelper.cpp
        ${_SRC_DIR}/layout/entities/robot/RobotAI.cpp
        ${_SRC_DIR}/layout/entities/coin/Coin.cpp
        ${_SRC_DIR}/layout/entities/coin/CoinHandler.cpp
        ${_SRC_DIR}/layout/entities/coin/animation/CoinCollectAnimEndCb.cpp
        ${_SRC_DIR}/layout/entities/coin/animation/CoinRespawnAnim.cpp
        ${_SRC_DIR}/layout/panels/PanelHandler.cpp
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
    PUBLIC
        robo_collector_common::robo_collector_common
        ros2_game_engine::ros2_game_engine
)

ament_target_dependencies(
    ${PROJECT_NAME}
    PUBLIC
        robo_collector_interfaces
        rclcpp
        ament_index_cpp
)

if(NOT DISABLE_ROS_TOOLING)
    # Ament uses non-monolith build (separate build and install steps).
    # The helpers.cmake has to be included manually.
    # This allows access to the helpers.cmake user defined macros
    # In the case of monolith build - the file is automatically included
    # from the cmake_helpersConfig.cmake when the package is consumed by find_package()
	include(${cmake_helpers_DIR}/helpers.cmake)
	
    enable_ros_tooling_for_target(
        ${PROJECT_NAME}
        ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
    )
endif()

set_target_cpp_standard(${PROJECT_NAME} 20)
enable_target_warnings(${PROJECT_NAME})
set_target_visibility(${PROJECT_NAME})
#enable_target_sanitizer(${PROJECT_NAME} "address")

# Install resources and auto-generated headers and resource bin files
install(
    DIRECTORY ${_RESOURCE_FOLDER_NAME}/
    DESTINATION share/${PROJECT_NAME}/${_RESOURCE_FOLDER_NAME}/
)

