#ifndef ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERCONFIG_H_
#define ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERCONFIG_H_

//System headers
#include <cstdint>

//Other libraries headers

//Own components headers
#include "robo_collector_controller/layout/config/RoboCollectorControllerLayoutConfig.h"
#include "robo_collector_controller/external_api/config/CollectorGuiExternalBridgeConfig.h"

//Forward declarations

struct RoboCollectorControllerConfig {
  RoboCollectorControllerLayoutConfig layoutCfg;
  CollectorGuiExternalBridgeConfig externalBridgeConfig;
};

#endif /* ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERCONFIG_H_ */

