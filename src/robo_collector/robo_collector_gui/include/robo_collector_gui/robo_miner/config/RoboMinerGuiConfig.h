#ifndef ROBO_MINER_CONFIG_ROBOMINERGUICONFIG_H_
#define ROBO_MINER_CONFIG_ROBOMINERGUICONFIG_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers
#include "robo_collector_gui/robo_miner/field/config/RoboMinerFieldConfig.h"

//Forward declarations

struct RoboMinerGuiConfig {
  RoboMinerFieldConfig fieldCfg;
  uint64_t crystalRsrcId = 0;
};

#endif /* ROBO_MINER_CONFIG_ROBOMINERGUICONFIG_H_ */
