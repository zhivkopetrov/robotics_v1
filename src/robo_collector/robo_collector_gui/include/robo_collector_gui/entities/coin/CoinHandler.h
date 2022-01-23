#ifndef ROBO_COLLECTOR_GUI_COIN_COINHANDLER_H_
#define ROBO_COLLECTOR_GUI_COIN_COINHANDLER_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>

//Other libraries headers

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiFunctionalDefines.h"
#include "robo_collector_gui/entities/coin/Coin.h"

//Forward declarations

struct CoinHandlerConfig {
  std::vector<uint64_t> animRsrcIds;
  std::vector<char> fieldDataMarkers;
  int32_t maxCoins = 0;
  int32_t rotateAnimFirstTimerId = 0;
  int32_t collectAnimFirstTimerId = 0;
  int32_t respawnAnimFirstTimerId = 0;
  char fieldEmptyDataMarker = '!';

  SetFieldDataMarkerCb setFieldDataMarkerCb;
  ResetFieldDataMarkerCb resetFieldDataMarkerCb;
  GetFieldDataCb getFieldDataCb;
  IncrCollectedCoinsCb incrCollectedCoinsCb;

  CollisionWatcher* collisionWatcher = nullptr;
};

class CoinHandler {
public:
  int32_t init(const CoinHandlerConfig& cfg);
  void deinit();
  void draw() const;

private:
  std::vector<Coin> _coins;
};

#endif /* ROBO_COLLECTOR_GUI_COIN_COINHANDLER_H_ */
