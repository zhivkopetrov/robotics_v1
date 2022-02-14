#author Zhivko Petrov

cmake_minimum_required(VERSION 3.10.2)

project(
    robo_common
    LANGUAGES CXX
    VERSION 1.0.0
)

list(
    APPEND
    CMAKE_MODULE_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake_helpers/find_modules
)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake_helpers/helpers.cmake)

find_package(manager_utils REQUIRED)

set(_INC_FOLDER_NAME include)
set(_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${_INC_FOLDER_NAME}/${PROJECT_NAME})       
set(_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_library(
    ${PROJECT_NAME} 
    STATIC
        ${_INC_DIR}/defines/RoboCommonDefines.h
        ${_INC_DIR}/layout/field/Field.h
        ${_INC_DIR}/layout/field/FieldPos.h
        ${_INC_DIR}/layout/field/FieldUtils.h
        ${_INC_DIR}/layout/field/Tile.h
        ${_INC_DIR}/layout/field/config/FieldConfig.h
        ${_INC_DIR}/layout/field/config/TileConfig.h
        ${_INC_DIR}/layout/panels/TimePanel.h
        ${_INC_DIR}/layout/panels/IndicatorPanel.h
        ${_INC_DIR}/layout/panels/config/IndicatorPanelConfig.h
        ${_INC_DIR}/layout/panels/config/TimePanelConfig.h
        ${_INC_DIR}/layout/entities/robot/Robot.h
        ${_INC_DIR}/layout/entities/robot/RobotInitHelper.h
        ${_INC_DIR}/layout/entities/robot/RobotUtils.h
        ${_INC_DIR}/layout/entities/robot/config/RobotBaseConfig.h
        ${_INC_DIR}/layout/entities/robot/animation/RobotAnimator.h
        ${_INC_DIR}/layout/entities/robot/animation/RobotAnimEndCb.h
        ${_INC_DIR}/layout/entities/robot/animation/PlayerDamageAnimEndCb.h
        ${_INC_DIR}/layout/animators/GameEndAnimator.h
        ${_INC_DIR}/helpers/CollisionObject.h
        ${_INC_DIR}/helpers/CollisionWatcher.h
        
        ${_SRC_DIR}/layout/field/Field.cpp
        ${_SRC_DIR}/layout/field/FieldPos.cpp
        ${_SRC_DIR}/layout/field/FieldUtils.cpp
        ${_SRC_DIR}/layout/field/Tile.cpp
        ${_SRC_DIR}/layout/panels/TimePanel.cpp
        ${_SRC_DIR}/layout/panels/IndicatorPanel.cpp
        ${_SRC_DIR}/layout/entities/robot/Robot.cpp
        ${_SRC_DIR}/layout/entities/robot/RobotInitHelper.cpp
        ${_SRC_DIR}/layout/entities/robot/RobotUtils.cpp
        ${_SRC_DIR}/layout/entities/robot/animation/RobotAnimator.cpp
        ${_SRC_DIR}/layout/entities/robot/animation/RobotAnimEndCb.cpp
        ${_SRC_DIR}/layout/entities/robot/animation/PlayerDamageAnimEndCb.cpp
        ${_SRC_DIR}/layout/animators/GameEndAnimator.cpp
        ${_SRC_DIR}/helpers/CollisionWatcher.cpp
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
        manager_utils::manager_utils
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

#invoke include-what-you-use (if it's enabled) for the current project
if(USE_IWYU)
    enable_target_include_what_you_use(${PROJECT_NAME})
endif(USE_IWYU)

if(BUILD_LIKE_POSITION_INDEPENDANT_CODE)
    enable_target_position_independent_code(${PROJECT_NAME})
endif()  




          



