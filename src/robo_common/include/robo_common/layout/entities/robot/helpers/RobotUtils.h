#ifndef ROBO_COMMON_ROBOTUTILS_H_
#define ROBO_COMMON_ROBOTUTILS_H_

//System headers
#include <cstdint>
#include <array>

//Other libraries headers

//Own components headers
#include "robo_common/defines/RoboCommonDefines.h"

//Forward declarations

class RobotUtils {
public:
  RobotUtils() = delete;

  static Direction getDirAfterRotation(Direction currDir, RotationDir rotDir);

  static double getRotationDegFromDir(Direction dir);

  static SurroundingTiles getSurroundingTiles(const FieldDescription &descr,
                                              const RobotState &state);
};

#endif /* ROBO_COMMON_ROBOTUTILS_H_ */
