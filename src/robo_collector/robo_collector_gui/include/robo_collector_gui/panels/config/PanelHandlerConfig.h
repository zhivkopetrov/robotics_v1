#ifndef ROBO_COLLECTOR_GUI_PANELS_PANELHANDLERCONFIG_H_
#define ROBO_COLLECTOR_GUI_PANELS_PANELHANDLERCONFIG_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers
#include "robo_collector_gui/panels/config/TimePanelConfig.h"
#include "robo_collector_gui/panels/config/CoinPanelConfig.h"
#include "robo_collector_gui/panels/config/HealthPanelConfig.h"

struct PanelHandlerConfig {
  TimePanelConfig timePanelCfg;
  HealthPanelConfig healthPanelCfg;
  CoinPanelConfig coinPanelCfg;
};

#endif /* ROBO_COLLECTOR_GUI_PANELS_PANELHANDLERCONFIG_H_ */
