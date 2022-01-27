#ifndef ROBO_COLLECTOR_GUI_PANELS_PANELHANDLER_H_
#define ROBO_COLLECTOR_GUI_PANELS_PANELHANDLER_H_

//C system headers

//C++ system headers

//Other libraries headers

//Own components headers
#include "robo_collector_gui/panels/config/PanelHandlerConfig.h"
#include "robo_collector_gui/panels/TimePanel.h"
#include "robo_collector_gui/panels/CoinPanel.h"
#include "robo_collector_gui/panels/HealthPanel.h"

//Forward declarations

class PanelHandler {
public:
  int32_t init(const PanelHandlerConfig& cfg);
  void draw() const;
  void decreaseHealthIndicator(int32_t damage);
  void increaseCollectedCoins(int32_t coins);

private:
  TimePanel _timePanel;
  CoinPanel _coinPanel;
  HealthPanel _healthPanel;
};

#endif /* ROBO_COLLECTOR_GUI_PANELS_PANELHANDLER_H_ */
