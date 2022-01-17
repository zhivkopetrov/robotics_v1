//Corresponding header
#include "robo_collector_gui/panels/Panel.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t Panel::init(const PanelConfig &cfg) {
  constexpr auto panelX = 1250;
  _panels[TIME_PANEL].create(cfg.timePanelRsrcId);
  _panels[TIME_PANEL].setPosition(panelX, 50);

  _panels[COIN_PANEL].create(cfg.coinPanelRsrcId);
  _panels[COIN_PANEL].setPosition(panelX, 215);

  _panels[HEALTH_PANEL].create(cfg.healthPanelRsrcId);
  _panels[HEALTH_PANEL].setPosition(panelX, 410);

  _healthIndicator.create(cfg.healthIndicatorRsrcId);
  _healthIndicator.setPosition(panelX + 79, 423);
  _healthIndicator.setCropRect(_healthIndicator.getCropRect());

  return SUCCESS;
}

void Panel::draw() const {
  for (const auto& panel : _panels) {
    panel.draw();
  }

  _healthIndicator.draw();
}

void Panel::shrinkHealthIndicator(int32_t deltaPx) {
  auto cropRectangle = _healthIndicator.getCropRect();
  cropRectangle.w -= deltaPx;
  _healthIndicator.setCropRect(cropRectangle);
}
