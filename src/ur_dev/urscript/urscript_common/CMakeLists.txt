# Author Zhivko Petrov

cmake_minimum_required(VERSION 3.10)

project(
    urscript_common
    LANGUAGES CXX
    VERSION 1.0.0
)

find_package(ament_cmake REQUIRED)
find_package(cmake_helpers REQUIRED)
find_package(utils REQUIRED)
find_package(urscript_interfaces REQUIRED)

set(_INC_FOLDER_NAME include)
set(_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${_INC_FOLDER_NAME}/${PROJECT_NAME})
set(_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_library(
    ${PROJECT_NAME}
    STATIC
        ${_INC_DIR}/defines/UrScriptTopics.h
        ${_INC_DIR}/defines/UrScriptDefines.h
        ${_INC_DIR}/defines/UrScriptStructs.h
        ${_INC_DIR}/message_helpers/UrScriptMessageHelpers.h
     
        ${_SRC_DIR}/defines/UrScriptStructs.cpp
        ${_SRC_DIR}/message_helpers/UrScriptMessageHelpers.cpp
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
        utils::utils
)

ament_export_targets(
    ${PROJECT_NAME}Targets 
        HAS_LIBRARY_TARGET
)

ament_target_dependencies(
    ${PROJECT_NAME}
    PUBLIC
        urscript_interfaces
)

ament_export_dependencies(
    ${PROJECT_NAME}
    PUBLIC
        urscript_interfaces
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



