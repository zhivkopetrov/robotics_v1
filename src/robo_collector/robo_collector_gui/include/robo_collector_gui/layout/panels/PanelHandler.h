#ifndef ROBO_COLLECTOR_GUI_PANELS_PANELHANDLER_H_
#define ROBO_COLLECTOR_GUI_PANELS_PANELHANDLER_H_

//System headers

//Other libraries headers
#include "robo_common/layout/panels/TimePanel.h"
#include "robo_common/layout/panels/IndicatorPanel.h"
#include "robo_common/layout/panels/NumberCounterPanel.h"

//Own components headers
#include "robo_collector_gui/layout/panels/config/PanelHandlerConfig.h"

//Forward declarations

struct PanelHandlerOutInterface {
  StartGameLostAnimCb startGameLostAnimCb;
  StartGameWonAnimCb startGameWonAnimCb;
  StartAchievementWonAnimCb startAchievementWonAnimCb;
};

class PanelHandler {
public:
  ErrorCode init(const PanelHandlerConfig &cfg,
                 const PanelHandlerOutInterface &interface);
  void draw() const;
  void decreaseHealthIndicator(int32_t damage);
  void increaseCollectedCoins(int32_t coins);

private:
  ErrorCode validateInterface(const PanelHandlerOutInterface &interface) const;

  TimePanel _timePanel;
  NumberCounterPanel _coinPanel;
  IndicatorPanel _healthPanel;
};

#endif /* ROBO_COLLECTOR_GUI_PANELS_PANELHANDLER_H_ */
