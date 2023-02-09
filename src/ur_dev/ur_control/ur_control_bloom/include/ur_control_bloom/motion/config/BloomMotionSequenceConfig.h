#ifndef UR_CONTROL_BLOOM_BLOOMMOTIONSEQUENCECONFIG_H_
#define UR_CONTROL_BLOOM_BLOOMMOTIONSEQUENCECONFIG_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "urscript_common/motion/MotionStructs.h"

//Own components headers

//Forward declarations

struct BloomMotionSequenceConfig {
  WaypointJoint homeJoint;
  WaypointJoint graspJoint;
  WaypointJoint graspApproachJoint;
  WaypointJoint placeJoint;
  WaypointJoint placeApproachJoint;

  WaypointCartesian homeCartesian;
};

#endif /* UR_CONTROL_BLOOM_BLOOMMOTIONSEQUENCECONFIG_H_ */
