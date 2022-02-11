#ifndef ROBO_MINER_CONFIG_ROBOCLEANERGUICONFIG_H_
#define ROBO_MINER_CONFIG_ROBOCLEANERGUICONFIG_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers
#include "robo_collector_gui/robo_cleaner/panels/config/EnergyPanelConfig.h"

//Forward declarations

struct RoboCleanerGuiConfig {
  EnergyPanelConfig energyPanelCfg;
  uint64_t rubbishRsrcId = 0;
  uint64_t rubbishFontId = 0;
  uint64_t obstacleRsrcId = 0;
};

#endif /* ROBO_MINER_CONFIG_ROBOCLEANERGUICONFIG_H_ */
