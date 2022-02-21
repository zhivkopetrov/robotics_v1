#ifndef ROBO_COLLECTOR_GUI_COINPANELCONFIG_H_
#define ROBO_COLLECTOR_GUI_COINPANELCONFIG_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers

struct CoinPanelConfig {
  uint64_t targetCoins = 0;
  uint64_t rsrcId = 0;
  uint64_t fontId = 0;
  int32_t incrTimerId = 0;
  int32_t decrTimerId = 0;
};

#endif /* ROBO_COLLECTOR_GUI_COINPANELCONFIG_H_ */
