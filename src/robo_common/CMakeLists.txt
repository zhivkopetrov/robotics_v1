#author Zhivko Petrov

cmake_minimum_required(VERSION 3.10.2)

project(
    robo_common
    LANGUAGES CXX
    VERSION 1.0.0
)

find_package(ament_cmake REQUIRED)
find_package(cmake_helpers REQUIRED)
find_package(manager_utils REQUIRED)

set(_INC_FOLDER_NAME include)
set(_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${_INC_FOLDER_NAME}/${PROJECT_NAME})       
set(_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_library(
    ${PROJECT_NAME} 
    STATIC
        ${_INC_DIR}/defines/RoboCommonDefines.h
        ${_INC_DIR}/layout/RoboCommonLayout.h
        ${_INC_DIR}/layout/config/RoboCommonLayoutConfig.h
        ${_INC_DIR}/layout/field/Field.h
        ${_INC_DIR}/layout/field/FieldPos.h
        ${_INC_DIR}/layout/field/FieldUtils.h
        ${_INC_DIR}/layout/field/Tile.h
        ${_INC_DIR}/layout/field/ObjectApproachOverlay.h
        ${_INC_DIR}/layout/field/fog_of_war/FogOfWar.h
        ${_INC_DIR}/layout/field/config/FieldConfig.h
        ${_INC_DIR}/layout/field/config/TileConfig.h
        ${_INC_DIR}/layout/field/config/FogOfWarConfig.h
        ${_INC_DIR}/layout/field/config/ObstacleHandlerConfig.h
        ${_INC_DIR}/layout/field/obstacles/ObstacleHandler.h
        ${_INC_DIR}/layout/field/obstacles/Obstacle.h
        ${_INC_DIR}/layout/panels/TimePanel.h
        ${_INC_DIR}/layout/panels/IndicatorPanel.h
        ${_INC_DIR}/layout/panels/config/IndicatorPanelConfig.h
        ${_INC_DIR}/layout/panels/config/TimePanelConfig.h
        ${_INC_DIR}/layout/panels/NumberCounterPanel.h
        ${_INC_DIR}/layout/panels/config/NumberCounterPanelConfig.h
        ${_INC_DIR}/layout/debug/DebugField.h
        ${_INC_DIR}/layout/debug/config/DebugFieldConfig.h
        ${_INC_DIR}/layout/entities/robot/Robot.h
        ${_INC_DIR}/layout/entities/robot/helpers/RobotActInterface.h
        ${_INC_DIR}/layout/entities/robot/helpers/RobotInitHelper.h
        ${_INC_DIR}/layout/entities/robot/helpers/RobotUtils.h
        ${_INC_DIR}/layout/entities/robot/config/RobotBaseConfig.h
        ${_INC_DIR}/layout/entities/robot/animation/RobotAnimator.h
        ${_INC_DIR}/layout/entities/robot/animation/RobotAnimEndCb.h
        ${_INC_DIR}/layout/entities/robot/animation/PlayerDamageAnimEndCb.h
        ${_INC_DIR}/layout/animators/GameEndAnimator.h
        ${_INC_DIR}/layout/animators/AchievementAnimator.h
        ${_INC_DIR}/layout/animators/EndScreenAppearAnimator.h
        ${_INC_DIR}/layout/animators/CountdownAnimator.h
        ${_INC_DIR}/layout/animators/HelpPageAnimator.h
        ${_INC_DIR}/layout/animators/config/GameEndAnimatorConfig.h
        ${_INC_DIR}/layout/animators/config/AchievementAnimatorConfig.h
        ${_INC_DIR}/layout/animators/config/HelpPageAnimatorConfig.h
        ${_INC_DIR}/layout/helpers/RoboCommonLayoutInitHelper.h
        ${_INC_DIR}/helpers/CollisionObject.h
        ${_INC_DIR}/helpers/CollisionWatcher.h
        ${_INC_DIR}/helpers/LevelFileLoader.h
        
        ${_SRC_DIR}/layout/RoboCommonLayout.cpp
        ${_SRC_DIR}/layout/field/Field.cpp
        ${_SRC_DIR}/layout/field/FieldPos.cpp
        ${_SRC_DIR}/layout/field/FieldUtils.cpp
        ${_SRC_DIR}/layout/field/Tile.cpp
        ${_SRC_DIR}/layout/field/ObjectApproachOverlay.cpp
        ${_SRC_DIR}/layout/field/fog_of_war/FogOfWar.cpp
        ${_SRC_DIR}/layout/field/fog_of_war/FogCollisionObject.cpp
        ${_SRC_DIR}/layout/field/obstacles/Obstacle.cpp
        ${_SRC_DIR}/layout/field/obstacles/ObstacleHandler.cpp
        ${_SRC_DIR}/layout/panels/TimePanel.cpp
        ${_SRC_DIR}/layout/panels/NumberCounterPanel.cpp
        ${_SRC_DIR}/layout/panels/IndicatorPanel.cpp
        ${_SRC_DIR}/layout/debug/DebugField.cpp
        ${_SRC_DIR}/layout/entities/robot/Robot.cpp
        ${_SRC_DIR}/layout/entities/robot/helpers/RobotInitHelper.cpp
        ${_SRC_DIR}/layout/entities/robot/helpers/RobotUtils.cpp
        ${_SRC_DIR}/layout/entities/robot/animation/RobotAnimator.cpp
        ${_SRC_DIR}/layout/entities/robot/animation/RobotAnimEndCb.cpp
        ${_SRC_DIR}/layout/entities/robot/animation/PlayerDamageAnimEndCb.cpp
        ${_SRC_DIR}/layout/animators/GameEndAnimator.cpp
        ${_SRC_DIR}/layout/animators/AchievementAnimator.cpp
        ${_SRC_DIR}/layout/animators/EndScreenAppearAnimator.cpp
        ${_SRC_DIR}/layout/animators/CountdownAnimator.cpp
        ${_SRC_DIR}/layout/animators/HelpPageAnimator.cpp
        ${_SRC_DIR}/layout/helpers/RoboCommonLayoutInitHelper.cpp
        ${_SRC_DIR}/helpers/CollisionWatcher.cpp
        ${_SRC_DIR}/helpers/LevelFileLoader.cpp
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

ament_export_targets(
    ${PROJECT_NAME}Targets
        HAS_LIBRARY_TARGET
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




          



