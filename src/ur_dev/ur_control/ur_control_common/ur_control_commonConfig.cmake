include(CMakeFindDependencyMacro)

#find_dependency will correctly forward REQUIRED or QUIET args to the consumer
#find_package is only for internal use
find_dependency(cmake_helpers REQUIRED)
find_dependency(ros2_game_engine REQUIRED)
find_dependency(ament_cmake REQUIRED)
find_dependency(rclcpp REQUIRED)
find_dependency(std_msgs REQUIRED)
find_dependency(std_srvs REQUIRED)
find_dependency(ur_dashboard_msgs REQUIRED)
find_dependency(urscript_interfaces REQUIRED)
find_dependency(urscript_common REQUIRED)

if(NOT TARGET ur_control_common::ur_control_common)
  include(${CMAKE_CURRENT_LIST_DIR}/ur_control_commonTargets.cmake)
endif()

# This is for catkin compatibility.
set(ur_control_common_LIBRARIES ur_control_common::ur_control_common)

get_target_property(
    ur_control_common_INCLUDE_DIRS
    ur_control_common::ur_control_common
    INTERFACE_INCLUDE_DIRECTORIES
)

