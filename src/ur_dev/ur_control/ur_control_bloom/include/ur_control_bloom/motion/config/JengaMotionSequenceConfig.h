#ifndef UR_CONTROL_BLOOM_JENGAMOTIONSEQUENCECONFIG_H_
#define UR_CONTROL_BLOOM_JENGAMOTIONSEQUENCECONFIG_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "urscript_common/motion/MotionStructs.h"

//Own components headers

//Forward declarations

struct JengaMotionSequenceConfig {
  WaypointJoint homeJoint;
  WaypointJoint graspApproachJoint;

  WaypointCartesian homeCartesian;
  WaypointCartesian graspApproachCartesian;
  WaypointCartesian baseCenterACartesian;
  WaypointCartesian baseCenterBCartesian;
};

#endif /* UR_CONTROL_BLOOM_JENGAMOTIONSEQUENCECONFIG_H_ */
