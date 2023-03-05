#author Zhivko Petrov

cmake_minimum_required(VERSION 3.10.2)

project(
    ur_control_common
    LANGUAGES CXX
    VERSION 1.0.0
)

find_package(ament_cmake REQUIRED)
find_package(cmake_helpers REQUIRED)
find_package(ros2_game_engine REQUIRED)
find_package(urscript_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ur_dashboard_msgs REQUIRED)

set(_INC_FOLDER_NAME include)
set(_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${_INC_FOLDER_NAME}/${PROJECT_NAME})       
set(_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_library(
    ${PROJECT_NAME} 
    STATIC
        ${_INC_DIR}/defines/UrControlCommonDefines.h
        ${_INC_DIR}/defines/UrControlCommonFunctionalDefines.h
        ${_INC_DIR}/defines/UrControlCommonTopics.h
        ${_INC_DIR}/external_api/config/UrContolCommonExternalBridgeConfig.h
        ${_INC_DIR}/external_api/UrControlCommonExternalBridge.h
        ${_INC_DIR}/external_api/DashboardProvider.h
        ${_INC_DIR}/layout/UrControlCommonLayout.h
        ${_INC_DIR}/layout/config/UrControlCommonLayoutConfig.h
        ${_INC_DIR}/layout/helpers/UrControlCommonLayoutInitHelper.h
        ${_INC_DIR}/layout/helpers/UrControlCommonLayoutInterfaces.h
        ${_INC_DIR}/layout/entities/robot/SafetyModeVisuals.h
        ${_INC_DIR}/layout/entities/button_handler/ButtonHandler.h
        ${_INC_DIR}/layout/entities/button_handler/UrScriptButtonHandler.h
        ${_INC_DIR}/layout/entities/button_handler/CustomActionButtonHandler.h
        ${_INC_DIR}/layout/entities/button_handler/ButtonHandlerInterfaces.h
        ${_INC_DIR}/layout/entities/button_handler/config/ButtonHandlerConfig.h
        ${_INC_DIR}/layout/entities/button_handler/config/UrScriptButtonHandlerConfig.h
        ${_INC_DIR}/layout/entities/button_handler/config/CustomActionButtonHandlerConfig.h
        ${_INC_DIR}/layout/entities/buttons/CommandButton.h
        ${_INC_DIR}/layout/entities/buttons/UrScriptButton.h
        ${_INC_DIR}/layout/entities/buttons/DashboardButton.h
        ${_INC_DIR}/layout/entities/buttons/CustomActionButton.h
        ${_INC_DIR}/motion/MotionSequence.h
        ${_INC_DIR}/motion/MotionSequenceExecutor.h
        ${_INC_DIR}/motion/MotionExecutor.h
        
        ${_SRC_DIR}/defines/UrControlCommonDefines.cpp
        ${_SRC_DIR}/external_api/UrControlCommonExternalBridge.cpp
        ${_SRC_DIR}/external_api/DashboardProvider.cpp
        ${_SRC_DIR}/layout/UrControlCommonLayout.cpp
        ${_SRC_DIR}/layout/helpers/UrControlCommonLayoutInitHelper.cpp
        ${_SRC_DIR}/layout/entities/robot/SafetyModeVisuals.cpp
        ${_SRC_DIR}/layout/entities/button_handler/ButtonHandler.cpp
        ${_SRC_DIR}/layout/entities/button_handler/UrScriptButtonHandler.cpp
        ${_SRC_DIR}/layout/entities/button_handler/CustomActionButtonHandler.cpp
        ${_SRC_DIR}/layout/entities/buttons/CommandButton.cpp
        ${_SRC_DIR}/layout/entities/buttons/UrScriptButton.cpp
        ${_SRC_DIR}/layout/entities/buttons/DashboardButton.cpp
        ${_SRC_DIR}/layout/entities/buttons/CustomActionButton.cpp
        ${_SRC_DIR}/motion/MotionSequence.cpp
        ${_SRC_DIR}/motion/MotionSequenceExecutor.cpp
        ${_SRC_DIR}/motion/MotionExecutor.cpp
)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(
    ${PROJECT_NAME} 
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${_INC_FOLDER_NAME}>
        $<INSTALL_INTERFACE:${_INC_FOLDER_NAME}>
)

target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC
        ros2_game_engine::ros2_game_engine
        urscript_common::urscript_common
)

ament_export_targets(
    ${PROJECT_NAME}Targets
        HAS_LIBRARY_TARGET
)

ament_target_dependencies(
    ${PROJECT_NAME}
    PUBLIC
        std_msgs
        std_srvs
        ur_dashboard_msgs
)

ament_export_dependencies(
    ${PROJECT_NAME}
    PUBLIC
        std_msgs
        std_srvs
        ur_dashboard_msgs
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

#invoke include-what-you-use (if it's enabled) for the current project
if(USE_IWYU)
    enable_target_include_what_you_use(${PROJECT_NAME})
endif(USE_IWYU)

if(BUILD_LIKE_POSITION_INDEPENDANT_CODE)
    enable_target_position_independent_code(${PROJECT_NAME})
endif()  




          



