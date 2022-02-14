#ifndef ROBO_COMMON_INDICATORPANEL_H_
#define ROBO_COMMON_INDICATORPANEL_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "manager_utils/drawing/Image.h"
#include "manager_utils/drawing/Text.h"
#include "manager_utils/time/TimerClient.h"

//Own components headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"
#include "robo_common/layout/panels/config/IndicatorPanelConfig.h"

//Forward declarations

class IndicatorPanel: public TimerClient {
public:
  int32_t init(const IndicatorPanelConfig &cfg,
               const IndicatorDepletedCb &indicatorDepletedCb);
  void draw() const;

  void decreaseIndicator(int32_t delta);

private:
  void onTimeout(const int32_t timerId) final;
  void processIndicatorReduceAnim();
  void setAndCenterIndicatorText();

  Image _panel;
  Image _indicator;
  Text _indicatorText;

  int32_t _indicatorReduceTimerId = 0;
  int32_t _damageTicksLeft = 0;

  IndicatorDepletedCb _indicatorDepletedCb;
};

#endif /* ROBO_COMMON_INDICATORPANEL_H_ */
