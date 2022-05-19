#ifndef ROBO_COMMON_ROBOCOMMONLAYOUTCONFIG_H_
#define ROBO_COMMON_ROBOCOMMONLAYOUTCONFIG_H_

//System headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "robo_common/layout/field/config/FieldConfig.h"
#include "robo_common/layout/field/config/FogOfWarConfig.h"
#include "robo_common/layout/entities/robot/config/RobotBaseConfig.h"

//Own components headers

//Forward declarations

struct RoboCommonLayoutConfig {
  FieldConfig fieldCfg;
  FogOfWarConfig fogOfWarConfig;
  RobotBaseConfig robotBaseCfg;

  uint64_t mapRsrcId = 0;

  char playerFieldMarker = 'B';
  char enemyFieldMarker = 'E';
};

#endif /* ROBO_COMMON_ROBOCOMMONLAYOUTCONFIG_H_ */
