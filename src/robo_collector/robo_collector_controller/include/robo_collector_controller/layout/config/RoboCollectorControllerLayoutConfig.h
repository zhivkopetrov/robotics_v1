#ifndef ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERLAYOUTCONFIG_H_
#define ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERLAYOUTCONFIG_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "robo_collector_common/layout/controller/config/RoboCollectorUiControllerBaseConfig.h"

//Own components headers

//Forward declarations

struct RoboCollectorControllerLayoutConfig {
  RoboCollectorUiControllerBaseConfig uiControllerCfg;
  uint64_t mapRsrcId = 0;
};

#endif /* ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERLAYOUTCONFIG_H_ */
