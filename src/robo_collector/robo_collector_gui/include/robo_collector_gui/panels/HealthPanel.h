#ifndef ROBO_COLLECTOR_GUI_HEALTHPANEL_H_
#define ROBO_COLLECTOR_GUI_HEALTHPANEL_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "manager_utils/drawing/Image.h"
#include "manager_utils/drawing/Text.h"
#include "manager_utils/time/TimerClient.h"

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiFunctionalDefines.h"
#include "robo_collector_gui/panels/config/HealthPanelConfig.h"

//Forward declarations

class HealthPanel: public TimerClient {
public:
  int32_t init(const HealthPanelConfig &cfg, const GameLostCb &gameLostCb);
  void draw() const;

  /* 1 damage == 1px */
  void decreaseHealthIndicator(int32_t damage);

private:
  void onTimeout(const int32_t timerId) final;
  void processIndicatorReduceAnim();
  void setAndCenterIndicatorText();

  Image _panel;
  Image _indicator;
  Text _indicatorText;

  int32_t _indicatorReduceTimerId = 0;
  int32_t _damageTicksLeft = 0;

  GameLostCb _gameLostCb;
};

#endif /* ROBO_COLLECTOR_GUI_HEALTHPANEL_H_ */
