#ifndef ROBO_COLLECTOR_GUI_PANELS_PANELCONFIG_H_
#define ROBO_COLLECTOR_GUI_PANELS_PANELCONFIG_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers

struct PanelConfig {
  uint64_t timePanelRsrcId = 0;
  uint64_t coinPanelRsrcId = 0;
  uint64_t healthPanelRsrcId = 0;
  uint64_t healthIndicatorRsrcId = 0;
  uint64_t horDelimiterRsrcId = 0;
  uint64_t vertDelimiterRsrcId = 0;
};

#endif /* ROBO_COLLECTOR_GUI_PANELS_PANELCONFIG_H_ */
