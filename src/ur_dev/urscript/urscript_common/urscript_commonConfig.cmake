include(CMakeFindDependencyMacro)

#find_dependency will correctly forward REQUIRED or QUIET args to the consumer
#find_package is only for internal use
find_dependency(cmake_helpers REQUIRED)
find_dependency(utils REQUIRED)

if(NOT TARGET urscript_common::urscript_common)
  include(${CMAKE_CURRENT_LIST_DIR}/urscript_commonTargets.cmake)
endif()

# This is for catkin compatibility.
set(urscript_common_LIBRARIES urscript_common::urscript_common)

get_target_property(
    urscript_common_INCLUDE_DIRS
    urscript_common::urscript_common
    INTERFACE_INCLUDE_DIRECTORIES
)

