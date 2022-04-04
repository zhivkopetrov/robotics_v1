include(CMakeFindDependencyMacro)

#find_dependency will correctly forward REQUIRED or QUIET args to the consumer
#find_package is only for internal use
find_dependency(cmake_helpers REQUIRED)
find_dependency(manager_utils REQUIRED)

if(NOT TARGET robo_common::robo_common)
  include(${CMAKE_CURRENT_LIST_DIR}/robo_commonTargets.cmake)
endif()

# This is for catkin compatibility.
set(robo_common_LIBRARIES robo_common::robo_common)

get_target_property(
    robo_common_INCLUDE_DIRS
    robo_common::robo_common
    INTERFACE_INCLUDE_DIRECTORIES
)

