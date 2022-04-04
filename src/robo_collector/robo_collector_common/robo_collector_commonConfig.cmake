include(CMakeFindDependencyMacro)

#find_dependency will correctly forward REQUIRED or QUIET args to the consumer
#find_package is only for internal use
find_dependency(cmake_helpers REQUIRED)
find_dependency(robo_common REQUIRED)

if(NOT TARGET robo_collector_common::robo_collector_common)
  include(${CMAKE_CURRENT_LIST_DIR}/robo_collector_commonTargets.cmake)
endif()

# This is for catkin compatibility.
set(robo_collector_common_LIBRARIES robo_collector_common::robo_collector_common)

get_target_property(
    robo_collector_common_INCLUDE_DIRS
    robo_collector_common::robo_collector_common
    INTERFACE_INCLUDE_DIRECTORIES
)

