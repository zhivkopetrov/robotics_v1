# Author Zhivko Petrov

cmake_minimum_required(VERSION 3.10)

project(
    robo_collector_common
    LANGUAGES CXX
    VERSION 1.0.0
)

find_package(cmake_helpers REQUIRED)
find_package(robo_common REQUIRED)
find_package(ament_cmake REQUIRED)
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

target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC
        robo_common::robo_common
)

ament_export_targets(
    ${PROJECT_NAME}Targets
        HAS_LIBRARY_TARGET
)

ament_target_dependencies(
    ${PROJECT_NAME}
    PUBLIC
        robo_collector_interfaces
)

ament_export_dependencies(
    ${PROJECT_NAME}
    PUBLIC
        robo_collector_interfaces
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
install_and_export_target(${PROJECT_NAME} ${_INC_FOLDER_NAME})



