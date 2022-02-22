#ifndef ROBO_COMMON_TIMEPANEL_H_
#define ROBO_COMMON_TIMEPANEL_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "manager_utils/drawing/Image.h"
#include "manager_utils/drawing/Text.h"
#include "manager_utils/time/TimerClient.h"

//Own components headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"
#include "robo_common/layout/panels/config/TimePanelConfig.h"

//Forward declarations

struct TimePanelUtilityConfig {
  IndicatorDepletedCb timeFinishedCb;
  Point pos;
};

class TimePanel: public TimerClient {
public:
  int32_t init(const TimePanelConfig &cfg,
               const TimePanelUtilityConfig &utilityCfg);
  void draw() const;

private:
  void onTimeout(const int32_t timerId) final;

  void processClockTick();

  void setTextContent();

  Color getStartColor(int32_t totalSeconds) const;

  Image _panel;
  Text _timeText;
  Rectangle _textBoundRect;

  int32_t _clockTimerId = 0;
  int32_t _blinkTimerId = 0;
  int32_t _remainingSeconds = 0;

  IndicatorDepletedCb _timeFinishedCb;
};

#endif /* ROBO_COMMON_TIMEPANEL_H_ */
