#ifndef ROBO_COMMON_ROBOTUTILS_H_
#define ROBO_COMMON_ROBOTUTILS_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers
#include "robo_common/defines/RoboCommonDefines.h"

//Forward declarations

class RobotUtils {
public:
  RobotUtils() = delete;

  static Direction getDirAfterRotation(Direction currDir, RotationDir rotDir);

  static double getRotationDegFromDir(Direction dir);
};

#endif /* ROBO_COMMON_ROBOTUTILS_H_ */
