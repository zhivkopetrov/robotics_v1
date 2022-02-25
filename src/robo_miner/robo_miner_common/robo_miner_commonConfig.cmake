include(CMakeFindDependencyMacro)

#find_dependency will correctly forward REQUIRED or QUIET args to the consumer
#find_package is only for internal use
find_dependency(robo_common REQUIRED)

if(NOT TARGET robo_miner_common::robo_miner_common)
  include(${CMAKE_CURRENT_LIST_DIR}/robo_miner_commonTargets.cmake)
endif()

# This is for catkin compatibility.
set(robo_miner_common_LIBRARIES robo_miner_common::robo_miner_common)

get_target_property(
    robo_miner_common_INCLUDE_DIRS
    robo_miner_common::robo_miner_common
    INTERFACE_INCLUDE_DIRECTORIES
)

