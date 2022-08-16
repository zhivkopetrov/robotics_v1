# Author Zhivko Petrov

cmake_minimum_required(VERSION 3.10)

project(
    urscript_bridge
    LANGUAGES CXX
    VERSION 1.0.0
)

find_package(cmake_helpers REQUIRED)
find_package(utils REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Threads REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ur_msgs REQUIRED)
find_package(urscript_interfaces REQUIRED)

set(_INC_FOLDER_NAME include)
set(_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${_INC_FOLDER_NAME}/${PROJECT_NAME})
set(_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_executable(
    ${PROJECT_NAME}
        ${_INC_DIR}/core/UrBridgeCommunicator.h
		${_INC_DIR}/core/UrBridgeAppilcation.h
        ${_INC_DIR}/external_api/UrBridgeExternalInterface.h
        ${_INC_DIR}/utils/TcpClient.h
     
		${_SRC_DIR}/core/UrBridgeCommunicator.cpp
		${_SRC_DIR}/core/UrBridgeAppilcation.cpp
		${_SRC_DIR}/external_api/UrBridgeExternalInterface.cpp
	    ${_SRC_DIR}/utils/TcpClient.cpp
        ${_SRC_DIR}/main.cpp
)

target_include_directories(
    ${PROJECT_NAME} 
    PRIVATE
        $<BUILD_INTERFACE:
            ${CMAKE_CURRENT_SOURCE_DIR}/${_INC_FOLDER_NAME}
        >
        $<INSTALL_INTERFACE:
            ${_INC_FOLDER_NAME}
        >
)

target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE
        utils::utils
        ${Boost_LIBRARIES}
        boost_thread #boosts linker flag for -lpthread
)

ament_target_dependencies(
    ${PROJECT_NAME}
    PUBLIC
		rclcpp
		std_msgs
		ur_msgs
		urscript_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

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
enable_target_sanitizer(${PROJECT_NAME} "thread")

