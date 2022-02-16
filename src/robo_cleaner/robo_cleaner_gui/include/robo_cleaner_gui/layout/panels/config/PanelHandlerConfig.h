#ifndef ROBO_CLEANER_GUI_PANELS_PANELHANDLERCONFIG_H_
#define ROBO_CLEANER_GUI_PANELS_PANELHANDLERCONFIG_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "robo_common/layout/panels/config/TimePanelConfig.h"
#include "robo_common/layout/panels/config/IndicatorPanelConfig.h"

//Own components headers

struct PanelHandlerConfig {
  IndicatorPanelConfig healthPanelCfg;
  IndicatorPanelConfig energyPanelCfg;
};

#endif /* ROBO_CLEANER_GUI_PANELS_PANELHANDLERCONFIG_H_ */
