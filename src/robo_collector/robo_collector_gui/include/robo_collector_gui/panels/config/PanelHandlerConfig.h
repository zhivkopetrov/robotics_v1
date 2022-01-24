#ifndef ROBO_COLLECTOR_GUI_PANELS_PANELHANDLERCONFIG_H_
#define ROBO_COLLECTOR_GUI_PANELS_PANELHANDLERCONFIG_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers

struct PanelHandlerConfig {
  uint64_t timePanelRsrcId = 0;
  uint64_t healthPanelRsrcId = 0;
  uint64_t healthIndicatorRsrcId = 0;
  uint64_t horDelimiterRsrcId = 0;
  uint64_t vertDelimiterRsrcId = 0;

  uint64_t coinPanelRsrcId = 0;
  uint64_t coinPanelFontId = 0;
  int32_t coinPanelIncrTimerId = 0;
  int32_t coinPanelDecrTimerId = 0;
};

#endif /* ROBO_COLLECTOR_GUI_PANELS_PANELHANDLERCONFIG_H_ */
