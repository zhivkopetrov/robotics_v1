#ifndef UR_CONTROL_BLOOM_PARKMOTIONSEQUENCECONFIG_H_
#define UR_CONTROL_BLOOM_PARKMOTIONSEQUENCECONFIG_H_

//System headers

//Other libraries headers
#include "urscript_common/motion/MotionStructs.h"

//Own components headers

//Forward declarations

struct ParkMotionSequenceConfig {
  double motionAcc = 0.0; //[m/s2]
  double motionVel = 0.0; //[m/s]

  WaypointJoint homeJoint;

  WaypointCartesian homeCartesian;
};

#endif /* UR_CONTROL_BLOOM_PARKMOTIONSEQUENCECONFIG_H_ */
