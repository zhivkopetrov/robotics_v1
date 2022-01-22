#ifndef ROBO_COLLECTOR_GUI_COIN_COINHANDLER_H_
#define ROBO_COLLECTOR_GUI_COIN_COINHANDLER_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>

//Other libraries headers

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiDefines.h"
#include "robo_collector_gui/defines/RoboCollectorGuiFunctionalDefines.h"
#include "robo_collector_gui/entities/coin/Coin.h"

//Forward declarations

struct CoinHandlerConfig {
  std::vector<uint64_t> animRsrcIds;
  int32_t maxCoins = 0;
  int32_t animFirstTimerId = 0;

  SetFieldDataMarkerCb setFieldDataMarkerCb;
  ResetFieldDataMarkerCb resetFieldDataMarkerCb;
  GetFieldDataCb getFieldDataCb;
};

class CoinHandler {
public:
  int32_t init(const CoinHandlerConfig& cfg);
  void draw() const;

private:
  std::vector<Coin> _coins;

  SetFieldDataMarkerCb _setFieldDataMarkerCb;
  ResetFieldDataMarkerCb _resetFieldDataMarkerCb;
  GetFieldDataCb _getFieldDataCb;
};

#endif /* ROBO_COLLECTOR_GUI_COIN_COINHANDLER_H_ */
