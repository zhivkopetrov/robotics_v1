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

set(_INC_FOLDER_NAME include)
set(_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${_INC_FOLDER_NAME}/${PROJECT_NAME})
set(_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_library(
    ${PROJECT_NAME}
    STATIC 
        ${_INC_DIR}/defines/RoboCollectorFunctionalDefines.h
        ${_INC_DIR}/layout/controller/buttons/MoveButton.h
        ${_INC_DIR}/layout/controller/buttons/HelpButton.h
        ${_INC_DIR}/layout/controller/buttons/SettingsButton.h
        ${_INC_DIR}/layout/controller/RoboCollectorController.h
        ${_INC_DIR}/layout/controller/config/RoboCollectorControllerConfig.h
     
        ${_SRC_DIR}/layout/controller/buttons/MoveButton.cpp
        ${_SRC_DIR}/layout/controller/buttons/HelpButton.cpp
        ${_SRC_DIR}/layout/controller/buttons/SettingsButton.cpp
        ${_SRC_DIR}/layout/controller/RoboCollectorController.cpp
)

target_include_directories(
    ${PROJECT_NAME} 
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${_INC_FOLDER_NAME}>
        $<INSTALL_INTERFACE:${_INC_FOLDER_NAME}>
)

target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC
        robo_common::robo_common
)

set_target_cpp_standard(${PROJECT_NAME} 20)
enable_target_warnings(${PROJECT_NAME})
set_target_visibility(${PROJECT_NAME})
install_and_export_target(${PROJECT_NAME} ${_INC_FOLDER_NAME})

if(NOT DISABLE_ROS_TOOLING)
    enable_ros_tooling_for_target(
        ${PROJECT_NAME}
        ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
    )
endif()



