# Author Zhivko Petrov

cmake_minimum_required(VERSION 3.10)

project(
    robo_cleaner_gui
    LANGUAGES CXX
    VERSION 1.0.0
)

find_package(cmake_helpers REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robo_cleaner_interfaces REQUIRED)
find_package(robo_cleaner_common REQUIRED)
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
        ${_INC_DIR}/helpers/EnergyHandler.h
        ${_INC_DIR}/helpers/RoboCleanerBuilder.h
        ${_INC_DIR}/helpers/RoboCleanerGuiInitHelper.h
        ${_INC_DIR}/helpers/MovementWatcher.h
        ${_INC_DIR}/helpers/RoboCleanerSolutionValidator.h
        ${_INC_DIR}/helpers/config/RoboCleanerSolutionValidatorConfig.h
        ${_INC_DIR}/helpers/config/EnergyHandlerConfig.h
        ${_INC_DIR}/external_api/CleanerControllerExternalBridge.h
        ${_INC_DIR}/external_api/MovementReporter.h
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
        ${_SRC_DIR}/external_api/CleanerControllerExternalBridge.cpp
        ${_SRC_DIR}/external_api/MovementReporter.cpp
        ${_SRC_DIR}/helpers/EnergyHandler.cpp
        ${_SRC_DIR}/helpers/RoboCleanerBuilder.cpp
        ${_SRC_DIR}/helpers/RoboCleanerGuiInitHelper.cpp
        ${_SRC_DIR}/helpers/MovementWatcher.cpp
        ${_SRC_DIR}/helpers/RoboCleanerSolutionValidator.cpp
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

target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC
        robo_cleaner_common::robo_cleaner_common
        ros2_game_engine::ros2_game_engine
)

ament_target_dependencies(
    ${PROJECT_NAME}
    PUBLIC
        robo_cleaner_interfaces
        rclcpp
        rclcpp_action
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

