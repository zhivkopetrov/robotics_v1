#ifndef ROBO_COMMON_ROBOCOMMONLAYOUTCONFIG_H_
#define ROBO_COMMON_ROBOCOMMONLAYOUTCONFIG_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "robo_common/layout/field/config/FieldConfig.h"
#include "robo_common/layout/entities/robot/config/RobotBaseConfig.h"

//Own components headers

//Forward declarations

struct RoboCommonLayoutConfig {
  FieldConfig fieldCfg;
  RobotBaseConfig robotBaseCfg;

  uint64_t mapRsrcId = 0;

  char playerFieldMarker = 'B';
  char enemyFieldMarker = 'E';
};

#endif /* ROBO_COMMON_ROBOCOMMONLAYOUTCONFIG_H_ */
