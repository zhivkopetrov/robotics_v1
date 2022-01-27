//Corresponding header
#include "robo_collector_gui/panels/HealthPanel.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t HealthPanel::init(const HealthPanelConfig& cfg) {
  constexpr auto panelX = 1250;
  _panel.create(cfg.rsrcId);
  _panel.setPosition(panelX, 390);

  _indicator.create(cfg.indicatorRsrcId);
  _indicator.setPosition(panelX + 79, 403);
  _indicator.setCropRect(_indicator.getImageRect());

  _indicatorReduceTimerId = cfg.indicatorReduceTimerId;

  return SUCCESS;
}

void HealthPanel::draw() const {
  _panel.draw();
  _indicator.draw();
}

void HealthPanel::decreaseHealthIndicator(int32_t damage) {
  auto cropRectangle = _indicator.getCropRect();
  cropRectangle.w -= damage;
  _indicator.setCropRect(cropRectangle);
}
