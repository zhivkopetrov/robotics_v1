# Author Zhivko Petrov

cmake_minimum_required(VERSION 3.10)

project(
    ur_control_gui
    LANGUAGES CXX
    VERSION 2.0.0
)

find_package(cmake_helpers REQUIRED)
find_package(ur_control_common)
find_package(ament_index_cpp REQUIRED)

set(_INC_FOLDER_NAME include)
set(_RESOURCE_FOLDER_NAME resources)
set(_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${_INC_FOLDER_NAME}/${PROJECT_NAME})
set(_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_executable(
    ${PROJECT_NAME} 
        ${_INC_DIR}/UrControlGui.h
        ${_INC_DIR}/config/UrControlGuiConfig.h
        ${_INC_DIR}/config/UrControlGuiConfigGenerator.h
        ${_INC_DIR}/external_api/UrControlGuiRos2ParamProvider.h
        ${_INC_DIR}/helpers/UrControlGuiBuilder.h
        ${_INC_DIR}/helpers/UrControlGuiInitHelper.h
     
        ${_SRC_DIR}/main.cpp
        ${_SRC_DIR}/UrControlGui.cpp
        ${_SRC_DIR}/config/UrControlGuiConfigGenerator.cpp
        ${_SRC_DIR}/external_api/UrControlGuiRos2ParamProvider.cpp
        ${_SRC_DIR}/helpers/UrControlGuiBuilder.cpp
        ${_SRC_DIR}/helpers/UrControlGuiInitHelper.cpp
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
        ur_control_common::ur_control_common
)

ament_target_dependencies(
    ${PROJECT_NAME}
    PUBLIC
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
# enable_target_sanitizer(${PROJECT_NAME} "address")

# Install resources and auto-generated headers and resource bin files
install(
    DIRECTORY ${_RESOURCE_FOLDER_NAME}/
    DESTINATION share/${PROJECT_NAME}/${_RESOURCE_FOLDER_NAME}/
)

