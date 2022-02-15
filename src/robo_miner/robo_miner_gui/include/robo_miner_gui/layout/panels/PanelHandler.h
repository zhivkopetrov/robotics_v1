#ifndef ROBO_MINER_GUI_PANELS_PANELHANDLER_H_
#define ROBO_MINER_GUI_PANELS_PANELHANDLER_H_

//C system headers

//C++ system headers

//Other libraries headers
#include "robo_common/layout/panels/TimePanel.h"
#include "robo_common/layout/panels/IndicatorPanel.h"

//Own components headers
#include "robo_miner_gui/layout/panels/config/PanelHandlerConfig.h"

//Forward declarations

struct PanelHandlerOutInterface {
  GameLostCb gameLostCb;
  GameWonCb gameWonCb;
};

class PanelHandler {
public:
  int32_t init(const PanelHandlerConfig &cfg,
               const PanelHandlerOutInterface &interface);
  void draw() const;
  void decreaseHealthIndicator(int32_t damage);

private:
  IndicatorPanel _healthPanel;
};

#endif /* ROBO_MINER_GUI_PANELS_PANELHANDLER_H_ */
