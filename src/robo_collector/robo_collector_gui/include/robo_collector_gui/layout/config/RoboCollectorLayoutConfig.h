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

//Forward declarations

struct RoboCollectorLayoutConfig {
  RoboCommonLayoutConfig commonLayoutCfg;

  PanelHandlerConfig panelHandlerCfg;
  CoinHandlerConfig coinHandlerCfg;
  RoboCollectorControllerConfig controllerCfg;
};

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUTCONFIG_H_ */
