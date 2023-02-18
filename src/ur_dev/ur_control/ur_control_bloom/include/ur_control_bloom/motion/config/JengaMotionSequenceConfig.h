#ifndef UR_CONTROL_BLOOM_JENGAMOTIONSEQUENCECONFIG_H_
#define UR_CONTROL_BLOOM_JENGAMOTIONSEQUENCECONFIG_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "urscript_common/motion/MotionStructs.h"

//Own components headers

//Forward declarations

struct JengaBlockDimensions {
  double width { };
  double depth { };
  double height { };
};

struct JengaMotionSequenceConfig {
  JengaBlockDimensions jengaBlockDimensions;
  AngleAxis zeroOrientation;   //neutral orientation
  AngleAxis ninetyOrientation; //neutral orientation + 90 deg on wrist_3

  WaypointJoint homeJoint;
  WaypointJoint graspApproachJoint;

  WaypointCartesian homeCartesian;
  WaypointCartesian graspApproachCartesian;
  WaypointCartesian baseCenterACartesian;
  WaypointCartesian baseCenterBCartesian;
};

#endif /* UR_CONTROL_BLOOM_JENGAMOTIONSEQUENCECONFIG_H_ */
