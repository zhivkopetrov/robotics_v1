#ifndef ROBO_COLLECTOR_GUI_COINHANDLERCONFIG_H_
#define ROBO_COLLECTOR_GUI_COINHANDLERCONFIG_H_

//System headers
#include <cstdint>
#include <vector>

//Other libraries headers

//Own components headers

//Forward declarations

struct CoinHandlerConfig {
  std::vector<uint64_t> animRsrcIds;
  std::vector<char> fieldMarkers;
  uint64_t targetWinCoins = 0;
  int32_t maxCoins = 0;
  int32_t rotateAnimFirstTimerId = 0;
  int32_t collectAnimFirstTimerId = 0;
  int32_t respawnAnimFirstTimerId = 0;
  char fieldEmptyMarker = '!';
};

#endif /* ROBO_COLLECTOR_GUI_COINHANDLERCONFIG_H_ */
