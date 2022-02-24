# Author Zhivko Petrov

cmake_minimum_required(VERSION 3.10)

project(
    robo_collector_common
    LANGUAGES CXX
    VERSION 1.0.0
)

list(
    APPEND
    CMAKE_MODULE_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake_helpers/find_modules
)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake_helpers/helpers.cmake)

find_package(robo_common REQUIRED)
find_package(robo_collector_interfaces REQUIRED)

set(_INC_FOLDER_NAME include)
set(_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${_INC_FOLDER_NAME}/${PROJECT_NAME})
set(_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_library(
    ${PROJECT_NAME}
    STATIC
        ${_INC_DIR}/defines/RoboCollectorTopics.h
        ${_INC_DIR}/defines/RoboCollectorFunctionalDefines.h
        ${_INC_DIR}/layout/controller/buttons/MoveButton.h
        ${_INC_DIR}/layout/controller/buttons/HelpButton.h
        ${_INC_DIR}/layout/controller/buttons/SettingsButton.h
        ${_INC_DIR}/layout/controller/RoboCollectorUiController.h
        ${_INC_DIR}/layout/controller/config/RoboCollectorUiControllerBaseConfig.h
        ${_INC_DIR}/message_helpers/RoboCollectorMessageHelpers.h
     
        ${_SRC_DIR}/layout/controller/buttons/MoveButton.cpp
        ${_SRC_DIR}/layout/controller/buttons/HelpButton.cpp
        ${_SRC_DIR}/layout/controller/buttons/SettingsButton.cpp
        ${_SRC_DIR}/layout/controller/RoboCollectorUiController.cpp
        ${_SRC_DIR}/message_helpers/RoboCollectorMessageHelpers.cpp
)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${_INC_FOLDER_NAME}>
        $<INSTALL_INTERFACE:${_INC_FOLDER_NAME}>
)

set(_USE_KEYWORD_STYLE False)
#note: ROS2 uses plain style, thus access modifiers can't be used
target_link_libraries(
    ${PROJECT_NAME}
        robo_common::robo_common
)

ament_target_dependencies(
    ${PROJECT_NAME}
        robo_collector_interfaces
)

set_target_cpp_standard(${PROJECT_NAME} 20)
enable_target_warnings(${PROJECT_NAME})
set_target_visibility(${PROJECT_NAME} ${_USE_KEYWORD_STYLE})
install_and_export_target(${PROJECT_NAME} ${_INC_FOLDER_NAME})

if(NOT DISABLE_ROS_TOOLING)
    enable_ros_tooling_for_target(
        ${PROJECT_NAME}
        ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
    )
endif()



