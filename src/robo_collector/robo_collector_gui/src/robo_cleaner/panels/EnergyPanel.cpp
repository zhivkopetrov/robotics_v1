//Corresponding header
#include "robo_collector_gui/robo_cleaner/panels/EnergyPanel.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/drawing/WidgetAligner.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t EnergyPanel::init(const EnergyPanelConfig &cfg) {
  constexpr auto panelX = 1250;
  constexpr auto panelY = 485;
  _panel.create(cfg.rsrcId);
  _panel.setPosition(panelX, panelY);

  _indicator.create(cfg.indicatorRsrcId);
  _indicator.setPosition(panelX + 79, panelY + 13);
  _indicator.setCropRect(_indicator.getImageRect());

  return SUCCESS;
}

void EnergyPanel::draw() const {
  _panel.draw();
  _indicator.draw();
}
