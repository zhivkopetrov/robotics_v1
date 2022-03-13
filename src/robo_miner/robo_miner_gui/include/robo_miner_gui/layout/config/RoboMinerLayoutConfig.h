#ifndef ROBO_MINER_GUI_ROBOMINERLAYOUTCONFIG_H_
#define ROBO_MINER_GUI_ROBOMINERLAYOUTCONFIG_H_

//System headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "robo_common/layout/config/RoboCommonLayoutConfig.h"

//Own components headers
#include "robo_miner_gui/layout/panels/config/PanelHandlerConfig.h"

//Forward declarations

struct RoboMinerLayoutConfig {
  RoboCommonLayoutConfig commonLayoutCfg;

  PanelHandlerConfig panelHandlerCfg;
  uint64_t crystalRsrcId = 0;
};

#endif /* ROBO_MINER_GUI_ROBOMINERLAYOUTCONFIG_H_ */
