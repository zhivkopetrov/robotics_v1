# Author Zhivko Petrov

cmake_minimum_required(VERSION 3.10)

project(
    robo_collector_controller
    LANGUAGES CXX
    VERSION 1.0.0
)

find_package(cmake_helpers REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robo_collector_interfaces REQUIRED)
find_package(robo_collector_common REQUIRED)
find_package(ros2_game_engine REQUIRED)

set(_INC_FOLDER_NAME include)
set(_RESOURCE_FOLDER_NAME resources)
set(_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${_INC_FOLDER_NAME}/${PROJECT_NAME})
set(_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_executable(
    ${PROJECT_NAME} 
        ${_INC_DIR}/RoboCollectorController.h
        ${_INC_DIR}/config/RoboCollectorControllerConfig.h
        ${_INC_DIR}/config/RoboCollectorControllerConfigGenerator.h
        ${_INC_DIR}/external_api/CollectorGuiExternalBridge.h
        ${_INC_DIR}/external_api/RoboCollectorControllerRos2ParamProvider.h
        ${_INC_DIR}/external_api/config/CollectorGuiExternalBridgeConfig.h
        ${_INC_DIR}/helpers/RoboCollectorControllerBuilder.h
        ${_INC_DIR}/helpers/RoboCollectorControllerInitHelper.h
        ${_INC_DIR}/layout/RoboCollectorControllerLayout.h
        ${_INC_DIR}/layout/config/RoboCollectorControllerLayoutConfig.h
        ${_INC_DIR}/layout/helpers/RoboCollectorControllerLayoutInterfaces.h
        ${_INC_DIR}/layout/helpers/RoboCollectorControllerLayoutInitHelper.h
     
        ${_SRC_DIR}/main.cpp
        ${_SRC_DIR}/RoboCollectorController.cpp
        ${_SRC_DIR}/config/RoboCollectorControllerConfigGenerator.cpp
        ${_SRC_DIR}/external_api/CollectorGuiExternalBridge.cpp
        ${_SRC_DIR}/external_api/RoboCollectorControllerRos2ParamProvider.cpp
        ${_SRC_DIR}/helpers/RoboCollectorControllerBuilder.cpp
        ${_SRC_DIR}/helpers/RoboCollectorControllerInitHelper.cpp
        ${_SRC_DIR}/layout/RoboCollectorControllerLayout.cpp
        ${_SRC_DIR}/layout/helpers/RoboCollectorControllerLayoutInitHelper.cpp
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
        rclcpp
        ament_index_cpp
        robo_collector_interfaces
        std_msgs
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
# enable_target_sanitizer(${PROJECT_NAME} "address")

# Install resources and auto-generated headers and resource bin files
install(
    DIRECTORY ${_RESOURCE_FOLDER_NAME}/
    DESTINATION share/${PROJECT_NAME}/${_RESOURCE_FOLDER_NAME}/
)

