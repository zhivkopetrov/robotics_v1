#ifndef ROBO_COMMON_NUMBERCOUNTERPANEL_H_
#define ROBO_COMMON_NUMBERCOUNTERPANEL_H_

//C system headers

//C++ system headers

//Other libraries headers
#include "manager_utils/drawing/NumberCounter.h"
#include "manager_utils/drawing/Text.h"

//Own components headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"
#include "robo_common/layout/panels/config/NumberCounterPanelConfig.h"

//Forward declarations

struct NumberCounterPanelUtilityConfig {
  NumberCounterTargetReachedCb targetReachedCb;
  Point pos;
  Color textColor = Colors::BLACK;
};

class NumberCounterPanel {
public:
  int32_t init(const NumberCounterPanelConfig& cfg,
               const NumberCounterPanelUtilityConfig &utilityCfg);
  void draw() const;
  void increaseCounter(int32_t delta);

  void onTargetCounterReached(uint64_t target);

private:
  NumberCounter _numberPanel;
  Text _totalCoinsText;

  NumberCounterTargetReachedCb _targetReachedCb;
};

#endif /* ROBO_COMMON_NUMBERCOUNTERPANEL_H_ */
