#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUTCONFIG_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUTCONFIG_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "robo_common/layout/config/RoboCommonLayoutConfig.h"

//Own components headers
#include "robo_collector_gui/layout/panels/config/PanelHandlerConfig.h"
#include "robo_collector_gui/layout/entities/coin/config/CoinHandlerConfig.h"
#include "robo_collector_gui/layout/controller/config/RoboCollectorControllerConfig.h"

#include "robo_collector_gui/layout/robo_miner/config/RoboMinerGuiConfig.h"
#include "robo_collector_gui/layout/robo_cleaner/config/RoboCleanerGuiConfig.h"

//Forward declarations

struct RoboCollectorLayoutConfig {
  RoboCommonLayoutConfig commonLayoutCfg;

  PanelHandlerConfig panelHandlerCfg;
  CoinHandlerConfig coinHandlerCfg;
  RoboCollectorControllerConfig controllerCfg;

  RoboMinerGuiConfig roboMinerGuiCfg;
  RoboCleanerGuiConfig roboCleanerGuiCfg;
};

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUTCONFIG_H_ */
