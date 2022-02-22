#ifndef ROBO_MINER_GUI_PANELS_PANELHANDLER_H_
#define ROBO_MINER_GUI_PANELS_PANELHANDLER_H_

//C system headers

//C++ system headers

//Other libraries headers
#include "robo_common/layout/panels/TimePanel.h"
#include "robo_common/layout/panels/IndicatorPanel.h"
#include "robo_common/layout/panels/NumberCounterPanel.h"

//Own components headers
#include "robo_miner_gui/layout/panels/config/PanelHandlerConfig.h"

//Forward declarations

struct PanelHandlerOutInterface {
  StartGameLostAnimCb startGameLostAnimCb;
  StartGameWonAnimCb startGameWonAnimCb;
  StartAchievementWonAnimCb startAchievementWonAnimCb;
};

class PanelHandler {
public:
  int32_t init(const PanelHandlerConfig &cfg,
               const PanelHandlerOutInterface &interface);
  void draw() const;
  void decreaseHealthIndicator(int32_t damage);

private:
  NumberCounterPanel _tilePanel;
  NumberCounterPanel _crystalPanel;
  IndicatorPanel _healthPanel;
};

#endif /* ROBO_MINER_GUI_PANELS_PANELHANDLER_H_ */
