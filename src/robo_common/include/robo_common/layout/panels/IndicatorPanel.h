#ifndef ROBO_COMMON_INDICATORPANEL_H_
#define ROBO_COMMON_INDICATORPANEL_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "manager_utils/drawing/Image.h"
#include "manager_utils/drawing/Text.h"
#include "manager_utils/time/TimerClient.h"

//Own components headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"
#include "robo_common/layout/panels/config/IndicatorPanelConfig.h"

//Forward declarations

inline constexpr int32_t INDICATOR_PANEL_MAX_VALUE = 400; //measured in px

struct IndicatorPanelUtilityConfig {
  IndicatorDepletedCb indicatorDepletedCb;
  Point pos;
};

class IndicatorPanel: public TimerClient {
public:
  ErrorCode init(const IndicatorPanelConfig &cfg,
                 const IndicatorPanelUtilityConfig& utilityCfg);
  void draw() const;

  void modifyIndicator(int32_t delta);

private:
  enum class IndicatorPanelAnimationType {
    INCREASE, DECREASE
  };

  void onTimeout(const int32_t timerId) final;
  void processIndicatorIncreaseAnim();
  void processIndicatorReduceAnim();
  void setAndCenterIndicatorText();

  Image _panel;
  Image _indicator;
  Text _indicatorText;

  int32_t _indicatorModifyTimerId = 0;

  //negative values are used in combination with
  //                                     IndicatorPanelAnimationType::DECREASE
  //positive values are used in combination with
  //                                     IndicatorPanelAnimationType::INCREASE
  int32_t _animTicksLeft = 0;

  IndicatorPanelAnimationType _currAnimType =
      IndicatorPanelAnimationType::DECREASE;

  IndicatorDepletedCb _indicatorDepletedCb;
};

#endif /* ROBO_COMMON_INDICATORPANEL_H_ */
