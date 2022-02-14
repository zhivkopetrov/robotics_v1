#ifndef ROBO_COLLECTOR_GUI_PANELS_PANELHANDLERCONFIG_H_
#define ROBO_COLLECTOR_GUI_PANELS_PANELHANDLERCONFIG_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "robo_common/panels/config/TimePanelConfig.h"
#include "robo_common/panels/config/IndicatorPanelConfig.h"

//Own components headers
#include "robo_collector_gui/panels/config/CoinPanelConfig.h"

struct PanelHandlerConfig {
  TimePanelConfig timePanelCfg;
  IndicatorPanelConfig healthPanelCfg;
  CoinPanelConfig coinPanelCfg;
};

#endif /* ROBO_COLLECTOR_GUI_PANELS_PANELHANDLERCONFIG_H_ */
