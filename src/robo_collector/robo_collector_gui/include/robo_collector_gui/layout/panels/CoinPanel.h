#ifndef ROBO_COLLECTOR_COINPANEL_H_
#define ROBO_COLLECTOR_COINPANEL_H_

//C system headers

//C++ system headers

//Other libraries headers
#include "manager_utils/drawing/NumberCounter.h"
#include "manager_utils/drawing/Text.h"

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiFunctionalDefines.h"
#include "robo_collector_gui/layout/panels/config/CoinPanelConfig.h"

//Forward declarations

class CoinPanel {
public:
  int32_t init(const CoinPanelConfig& cfg, const GameWonCb &gameWonCb);
  void draw() const;
  void increaseCollectedCoins(int32_t coins);

private:
  NumberCounter _numberPanel;
  Text _totalCoinsText;

  GameWonCb _gameWonCb;
};

#endif /* ROBO_COLLECTOR_COINPANEL_H_ */
