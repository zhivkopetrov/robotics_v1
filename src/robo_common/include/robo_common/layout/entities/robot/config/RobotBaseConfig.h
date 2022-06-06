#ifndef ROBO_COMMON_ROBOTBASECONFIG_H_
#define ROBO_COMMON_ROBOTBASECONFIG_H_

//System headers
#include <cstdint>

//Other libraries headers

//Own components headers
#include "robo_common/defines/RoboCommonDefines.h"

//Forward declarations

struct RobotBaseConfig {
  uint64_t playerRsrcId = 0;
  uint64_t enemiesRsrcId = 0;
  uint64_t damageMarkerRsrcId = 0;
  int32_t moveAnimStartTimerId = 0;
  int32_t rotateAnimStartTimerId = 0;
  int32_t robotCollisionAnimStartTimerId = 0;
  int32_t robotDamageAnimStartTimerId = 0;
  RobotFieldMarkers robotFieldMarkers = RobotFieldMarkers::DISABLED;
};

#endif /* ROBO_COMMON_ROBOTBASECONFIG_H_ */
