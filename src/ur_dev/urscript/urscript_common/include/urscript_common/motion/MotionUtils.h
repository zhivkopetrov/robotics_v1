#ifndef URSCRIPT_COMMON_MOTIONUTILS_H_
#define URSCRIPT_COMMON_MOTIONUTILS_H_

//System headers
#include <array>

//Other libraries headers

//Own components headers
#include "urscript_common/motion/MotionStructs.h"

//Forward declarations

double toRadians(double degrees);
double toDegrees(double radians);

double getDistance(const Point3d& lhs, const Point3d& rhs);

//returns blending radius equals to the 
//min[distance(startPos, midPos), distance(midPos, endPos)] * (100 - safetyDeltaPercent)
//safety delta is needed, because the robot might overshoot when using high 
//acceleration/velocity profile, leading to innacure blendings
double computeSafeBlendingRadius(
  const Point3d& startPos, const Point3d& midPos, const Point3d& endPos, 
  double safetyDeltaPercent = 10);

#endif /* URSCRIPT_COMMON_MOTIONUTILS_H_ */