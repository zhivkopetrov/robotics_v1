#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUTCONFIG_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUTCONFIG_H_

//System headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "robo_common/layout/config/RoboCommonLayoutConfig.h"
#include "robo_collector_common/layout/controller/config/RoboCollectorUiControllerBaseConfig.h"

//Own components headers
#include "robo_collector_gui/layout/panels/config/PanelHandlerConfig.h"
#include "robo_collector_gui/layout/entities/coin/config/CoinHandlerConfig.h"

//Forward declarations

struct RoboCollectorLayoutConfig {
  RoboCommonLayoutConfig commonLayoutCfg;

  PanelHandlerConfig panelHandlerCfg;
  CoinHandlerConfig coinHandlerCfg;
  RoboCollectorUiControllerBaseConfig controllerCfg;
};

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUTCONFIG_H_ */
