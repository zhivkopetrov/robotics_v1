#ifndef ROBO_COLLECTOR_GUI_COIN_COINHANDLER_H_
#define ROBO_COLLECTOR_GUI_COIN_COINHANDLER_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiFunctionalDefines.h"
#include "robo_collector_gui/entities/coin/config/CoinHandlerConfig.h"
#include "robo_collector_gui/entities/coin/Coin.h"

//Forward declarations

class CoinHandler {
public:
  int32_t init(const CoinHandlerConfig& cfg, const CoinOutInterface& interface);
  void deinit();
  void draw() const;

private:
  std::vector<Coin> _coins;
};

#endif /* ROBO_COLLECTOR_GUI_COIN_COINHANDLER_H_ */
