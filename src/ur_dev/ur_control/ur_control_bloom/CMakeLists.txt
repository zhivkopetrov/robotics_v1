# Author Zhivko Petrov

cmake_minimum_required(VERSION 3.10)

project(
    ur_control_bloom
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
        ${_INC_DIR}/UrControlBloom.h
        ${_INC_DIR}/defines/UrControlBloomDefines.h
        ${_INC_DIR}/config/UrControlBloomConfig.h
        ${_INC_DIR}/config/UrControlBloomConfigGenerator.h
        ${_INC_DIR}/external_api/UrControlBloomRos2ParamProvider.h
        ${_INC_DIR}/helpers/UrControlBloomBuilder.h
        ${_INC_DIR}/helpers/UrControlBloomInitHelper.h
        ${_INC_DIR}/layout/config/UrControlBloomLayoutConfig.h
        ${_INC_DIR}/layout/UrControlBloomLayout.h
        ${_INC_DIR}/motion/BloomMotionSequence.h
        ${_INC_DIR}/motion/config/BloomMotionSequenceConfig.h
     
        ${_SRC_DIR}/main.cpp
        ${_SRC_DIR}/UrControlBloom.cpp
        ${_SRC_DIR}/config/UrControlBloomConfigGenerator.cpp
        ${_SRC_DIR}/external_api/UrControlBloomRos2ParamProvider.cpp
        ${_SRC_DIR}/helpers/UrControlBloomBuilder.cpp
        ${_SRC_DIR}/helpers/UrControlBloomInitHelper.cpp
        ${_SRC_DIR}/layout/UrControlBloomLayout.cpp
        ${_SRC_DIR}/motion/BloomMotionSequence.cpp
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

