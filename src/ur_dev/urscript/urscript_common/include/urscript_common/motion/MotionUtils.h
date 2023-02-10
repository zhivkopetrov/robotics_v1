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

#endif /* URSCRIPT_COMMON_MOTIONUTILS_H_ */