#ifndef ROBO_COMMON_OBSTACLEHANDLERCONFIG_H_
#define ROBO_COMMON_OBSTACLEHANDLERCONFIG_H_

//System headers
#include <cstdint>

//Other libraries headers

//Own components headers
#include "robo_common/defines/RoboCommonDefines.h"

//Forward declarations

struct ObstacleHandlerConfig {
  uint64_t obstacleRsrcId = 0;
  ObstacleHandlerApproachOverlayStatus status =
      ObstacleHandlerApproachOverlayStatus::DISABLED;
};

#endif /* ROBO_COMMON_OBSTACLEHANDLERCONFIG_H_ */
