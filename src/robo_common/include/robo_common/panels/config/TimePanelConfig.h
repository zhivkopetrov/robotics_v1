#ifndef ROBO_COLLECTOR_GUI_TIMEPANELCONFIG_H_
#define ROBO_COLLECTOR_GUI_TIMEPANELCONFIG_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers

struct TimePanelConfig {
  uint64_t rsrcId = 0;
  uint64_t fontId = 0;
  int32_t clockTimerId = 0;
  int32_t blinkTimerId = 0;
  int32_t totalSeconds = 0;
};

#endif /* ROBO_COLLECTOR_GUI_TIMEPANELCONFIG_H_ */
