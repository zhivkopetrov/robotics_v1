#ifndef ROBO_COMMON_ROBOCOMMONLAYOUTCONFIG_H_
#define ROBO_COMMON_ROBOCOMMONLAYOUTCONFIG_H_

//System headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "robo_common/layout/field/config/FieldConfig.h"
#include "robo_common/layout/field/config/FogOfWarConfig.h"
#include "robo_common/layout/entities/robot/config/RobotBaseConfig.h"
#include "robo_common/layout/animators/config/GameEndAnimatorConfig.h"
#include "robo_common/layout/animators/config/AchievementAnimatorConfig.h"

//Own components headers

//Forward declarations

struct RoboCommonLayoutConfig {
  FieldConfig fieldCfg;
  FogOfWarConfig fogOfWarConfig;
  RobotBaseConfig robotBaseCfg;
  RobotState robotInitialState;
  GameEndAnimatorConfig gameEndAnimatorConfig;
  AchievementAnimatorConfig achievementAnimatorConfig;

  uint64_t mapRsrcId = 0;

  char playerFieldMarker = 'B';
  char enemyFieldMarker = 'E';
};

#endif /* ROBO_COMMON_ROBOCOMMONLAYOUTCONFIG_H_ */
