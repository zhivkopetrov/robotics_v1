#ifndef ROBO_MINER_GUI_ROBOMINERFIELDCONFIG_H_
#define ROBO_MINER_GUI_ROBOMINERFIELDCONFIG_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers

//Forward declarations

struct RoboMinerFieldConfig {
  int32_t rows = 0;
  int32_t cols = 0;
  char emptyTileMarker = '.';
};

#endif /* ROBO_MINER_GUI_ROBOMINERFIELDCONFIG_H_ */
