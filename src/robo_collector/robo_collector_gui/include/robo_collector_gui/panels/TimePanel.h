#ifndef ROBO_COLLECTOR_GUI_TIMEPANEL_H_
#define ROBO_COLLECTOR_GUI_TIMEPANEL_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "manager_utils/drawing/Image.h"
#include "manager_utils/drawing/Text.h"
#include "manager_utils/time/TimerClient.h"

//Own components headers
#include "robo_collector_gui/panels/config/TimePanelConfig.h"

//Forward declarations

class TimePanel : public TimerClient {
public:
  int32_t init(const TimePanelConfig& cfg);
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
};

#endif /* ROBO_COLLECTOR_GUI_TIMEPANEL_H_ */
