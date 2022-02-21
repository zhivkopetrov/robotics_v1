#ifndef ROBO_COLLECTOR_COINPANEL_H_
#define ROBO_COLLECTOR_COINPANEL_H_

//C system headers

//C++ system headers

//Other libraries headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"
#include "manager_utils/drawing/NumberCounter.h"
#include "manager_utils/drawing/Text.h"

//Own components headers
#include "robo_collector_gui/layout/panels/config/CoinPanelConfig.h"

//Forward declarations

class CoinPanel {
public:
  int32_t init(const CoinPanelConfig& cfg,
               const StartGameWonAnimCb &startGameWonAnimCb);
  void draw() const;
  void increaseCollectedCoins(int32_t coins);

  void onTargetCoinsReached(uint64_t targetCoins);

private:
  NumberCounter _numberPanel;
  Text _totalCoinsText;

  StartGameWonAnimCb _startGameWonAnimCb;
};

#endif /* ROBO_COLLECTOR_COINPANEL_H_ */
