#ifndef UR_CONTROL_BLOOM_BLOOMMOTIONSEQUENCECONFIG_H_
#define UR_CONTROL_BLOOM_BLOOMMOTIONSEQUENCECONFIG_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "urscript_common/motion/MotionStructs.h"

//Own components headers

//Forward declarations

struct BloomMotionSequenceConfig {
  double pickAndPlaceAcc = 0.0; //[m/s2]
  double pickAndPlaceVel = 0.0; //[m/s]

  WaypointJoint homeJoint;
  WaypointJoint graspJoint;
  WaypointJoint graspApproachJoint;
  WaypointJoint placeApproachJoint;

  WaypointCartesian homeCartesian;
  WaypointCartesian graspCartesian;
  WaypointCartesian graspApproachCartesian;
  WaypointCartesian placeCartesian;
  WaypointCartesian placeApproachCartesian;
};

#endif /* UR_CONTROL_BLOOM_BLOOMMOTIONSEQUENCECONFIG_H_ */
