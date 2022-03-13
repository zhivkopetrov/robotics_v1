#ifndef ROBO_COLLECTOR_GUI_COIN_COINHANDLER_H_
#define ROBO_COLLECTOR_GUI_COIN_COINHANDLER_H_

//System headers
#include <cstdint>

//Other libraries headers

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiFunctionalDefines.h"
#include "robo_collector_gui/layout/entities/coin/config/CoinHandlerConfig.h"
#include "robo_collector_gui/layout/entities/coin/Coin.h"

//Forward declarations

class CoinHandler {
public:
  ErrorCode init(
      const CoinHandlerConfig& cfg, const CoinOutInterface& interface);
  void deinit();
  void draw() const;

private:
  std::vector<Coin> _coins;
};

#endif /* ROBO_COLLECTOR_GUI_COIN_COINHANDLER_H_ */
