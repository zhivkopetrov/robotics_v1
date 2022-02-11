#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUICONFIG_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUICONFIG_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>

//Other libraries headers

//Own components headers
#include "robo_collector_gui/field/config/FieldConfig.h"
#include "robo_collector_gui/panels/config/PanelHandlerConfig.h"
#include "robo_collector_gui/entities/robot/config/RobotBaseConfig.h"
#include "robo_collector_gui/entities/coin/config/CoinHandlerConfig.h"
#include "robo_collector_gui/controller/config/RoboCollectorControllerConfig.h"

#include "robo_collector_gui/robo_miner/config/RoboMinerGuiConfig.h"
#include "robo_collector_gui/robo_cleaner/config/RoboCleanerGuiConfig.h"

//Forward declarations

struct RoboCollectorGuiConfig {
  FieldConfig fieldCfg;
  PanelHandlerConfig panelHandlerCfg;
  RobotBaseConfig robotBaseCfg;
  CoinHandlerConfig coinHandlerCfg;
  RoboCollectorControllerConfig controllerCfg;

  uint64_t mapRsrcId = 0;

  char playerFieldMarker = 'B';
  char enemyFieldMarker = 'E';

  RoboMinerGuiConfig roboMinerGuiConfig;
  RoboCleanerGuiConfig roboCleanerGuiConfig;
};

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUICONFIG_H_ */

