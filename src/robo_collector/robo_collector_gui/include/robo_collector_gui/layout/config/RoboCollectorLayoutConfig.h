#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUTCONFIG_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUTCONFIG_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "robo_common/layout/field/config/FieldConfig.h"
#include "robo_common/layout/entities/robot/config/RobotBaseConfig.h"

//Own components headers
#include "robo_collector_gui/panels/config/PanelHandlerConfig.h"
#include "robo_collector_gui/entities/coin/config/CoinHandlerConfig.h"
#include "robo_collector_gui/controller/config/RoboCollectorControllerConfig.h"

#include "robo_collector_gui/robo_miner/config/RoboMinerGuiConfig.h"
#include "robo_collector_gui/robo_cleaner/config/RoboCleanerGuiConfig.h"

//Forward declarations

struct RoboCollectorLayoutConfig {
  FieldConfig fieldCfg;
  PanelHandlerConfig panelHandlerCfg;
  RobotBaseConfig robotBaseCfg;
  CoinHandlerConfig coinHandlerCfg;
  RoboCollectorControllerConfig controllerCfg;

  uint64_t mapRsrcId = 0;

  char playerFieldMarker = 'B';
  char enemyFieldMarker = 'E';

  RoboMinerGuiConfig roboMinerGuiCfg;
  RoboCleanerGuiConfig roboCleanerGuiCfg;
};

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUTCONFIG_H_ */
