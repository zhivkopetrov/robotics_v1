# Author Zhivko Petrov

cmake_minimum_required(VERSION 3.10)

project(
    robo_cleaner_gui
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
find_package(robo_cleaner_interfaces REQUIRED)
find_package(robo_common REQUIRED)
find_package(ros2_game_engine REQUIRED)

set(_INC_FOLDER_NAME include)
set(_RESOURCE_FOLDER_NAME resources)
set(_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${_INC_FOLDER_NAME}/${PROJECT_NAME})
set(_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_executable(
    ${PROJECT_NAME} 
        ${_INC_DIR}/RoboCleanerGui.h
        ${_INC_DIR}/config/RoboCleanerGuiConfig.h
        ${_INC_DIR}/config/RoboCleanerGuiConfigGenerator.h
        ${_INC_DIR}/defines/RoboCleanerGuiDefines.h
        ${_INC_DIR}/defines/RoboCleanerGuiFunctionalDefines.h
        ${_INC_DIR}/helpers/RoboCleanerBuilder.h
        ${_INC_DIR}/helpers/RoboCleanerGuiInitHelper.h
        ${_INC_DIR}/external_api/CleanerControllerExternalBridge.h
        ${_INC_DIR}/layout/entities/Rubbish.h
        ${_INC_DIR}/layout/entities/EntityHandler.h
        ${_INC_DIR}/layout/entities/config/EntityHandlerConfig.h
        ${_INC_DIR}/layout/panels/PanelHandler.h
        ${_INC_DIR}/layout/panels/config/PanelHandlerConfig.h
        ${_INC_DIR}/layout/helpers/RoboCleanerLayoutInterfaces.h
        ${_INC_DIR}/layout/helpers/RoboCleanerLayoutInitHelper.h
     
        ${_SRC_DIR}/main.cpp
        ${_SRC_DIR}/RoboCleanerGui.cpp
        ${_SRC_DIR}/config/RoboCleanerGuiConfigGenerator.cpp
        ${_SRC_DIR}/defines/RoboCleanerGuiDefines.cpp
        ${_SRC_DIR}/external_api/CleanerControllerExternalBridge.cpp
        ${_SRC_DIR}/helpers/RoboCleanerBuilder.cpp
        ${_SRC_DIR}/helpers/RoboCleanerGuiInitHelper.cpp
        ${_SRC_DIR}/layout/RoboCleanerLayout.cpp
        ${_SRC_DIR}/layout/entities/Rubbish.cpp
        ${_SRC_DIR}/layout/entities/EntityHandler.cpp
        ${_SRC_DIR}/layout/panels/PanelHandler.cpp
        ${_SRC_DIR}/layout/helpers/RoboCleanerLayoutInitHelper.cpp
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

set(_USE_KEYWORD_STYLE False)
#note: ROS2 uses plain style, thus access modifiers can't be used
target_link_libraries(
    ${PROJECT_NAME}
        robo_common::robo_common
        ros2_game_engine::ros2_game_engine
)

ament_target_dependencies(
    ${PROJECT_NAME}
        robo_cleaner_interfaces
        rclcpp
        ament_index_cpp
)

set_target_cpp_standard(${PROJECT_NAME} 20)
enable_target_warnings(${PROJECT_NAME})
set_target_visibility(${PROJECT_NAME} ${_USE_KEYWORD_STYLE})
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

