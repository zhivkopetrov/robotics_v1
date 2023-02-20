//Corresponding header
#include "urscript_common/motion/MotionUtils.h"

//System headers
#include <cmath>

//Other libraries headers
#include "utils/Log.h"

//Own components headers

namespace {
constexpr double DEGREES_TO_RADIANS_MULTIPLIER = std::numbers::pi / 180.0;
constexpr double RADIANS_TO_DEGREES_MULTIPLIER = 180.0 / std::numbers::pi;
}

double toRadians(double degrees) {
  return degrees * DEGREES_TO_RADIANS_MULTIPLIER;
}

double toDegrees(double radians) {
  return radians * RADIANS_TO_DEGREES_MULTIPLIER;
}

double getDistance(const Point3d& lhs, const Point3d& rhs) {
  const double xDiff = (lhs.x - rhs.x);
  const double yDiff = (lhs.y - rhs.y);
  const double zDiff = (lhs.z - rhs.z);
  const double distSquared = 
    (xDiff * xDiff) + (yDiff * yDiff) + (zDiff * zDiff);

  return std::sqrt(distSquared);
}

double computeSafeBlendingRadius(
  const Point3d& startPos, const Point3d& midPos, const Point3d& endPos, 
  double safetyDeltaPercent) {
  if ((0.0 > safetyDeltaPercent) || (100.0 < safetyDeltaPercent)) {
    LOGR("Invalid safetyDeltaPercent provided: [%f]. Accepted range [0-100], "
         "Defaulting to 10", safetyDeltaPercent);
    safetyDeltaPercent = 10.0;
  }

  const double leftDist = getDistance(startPos, midPos);
  const double rightDist = getDistance(midPos, endPos);
  const double minDist = std::min(leftDist, rightDist);

  return minDist * ((100.0 - safetyDeltaPercent) / 100.0);
}
