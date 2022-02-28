#ifndef ROBO_MINER_GUI_CONFIG_ROBOMINERGUICONFIG_H_
#define ROBO_MINER_GUI_CONFIG_ROBOMINERGUICONFIG_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers
#include "robo_miner_gui/layout/config/RoboMinerLayoutConfig.h"
#include "robo_miner_gui/helpers/config/SolutionValidatorConfig.h"

//Forward declarations

struct RoboMinerGuiConfig {
  RoboMinerLayoutConfig layoutCfg;
  SolutionValidatorConfig solutionValidatorCfg;
};

#endif /* ROBO_MINER_GUI_CONFIG_ROBOMINERGUICONFIG_H_ */
