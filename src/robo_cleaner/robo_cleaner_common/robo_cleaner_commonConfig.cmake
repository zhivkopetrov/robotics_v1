include(CMakeFindDependencyMacro)

#find_dependency will correctly forward REQUIRED or QUIET args to the consumer
#find_package is only for internal use
find_dependency(cmake_helpers REQUIRED)
find_dependency(robo_common REQUIRED)
find_dependency(robo_cleaner_interfaces REQUIRED)

if(NOT TARGET robo_cleaner_common::robo_cleaner_common)
  include(${CMAKE_CURRENT_LIST_DIR}/robo_cleaner_commonTargets.cmake)
endif()

# This is for catkin compatibility.
set(robo_cleaner_common_LIBRARIES robo_cleaner_common::robo_cleaner_common)

get_target_property(
    robo_cleaner_common_INCLUDE_DIRS
    robo_cleaner_common::robo_cleaner_common
    INTERFACE_INCLUDE_DIRECTORIES
)

