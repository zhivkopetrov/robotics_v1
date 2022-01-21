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
  _timePanel.create(cfg.timePanelRsrcId);
  _timePanel.setPosition(panelX, 50);

  _healthPanel.create(cfg.healthPanelRsrcId);
  _healthPanel.setPosition(panelX, 390);

  _healthIndicator.create(cfg.healthIndicatorRsrcId);
  _healthIndicator.setPosition(panelX + 79, 403);
  _healthIndicator.setCropRect(_healthIndicator.getCropRect());

  _horDelimiter.create(cfg.horDelimiterRsrcId);
  _horDelimiter.setPosition(1245, 500);

  _vertDelimiter.create(cfg.vertDelimiterRsrcId);
  _vertDelimiter.setPosition(1200, 550);

  NumberCounterConfig coinCounterCfg;
  coinCounterCfg.backgroundRsrcId = cfg.coinPanelRsrcId;
  coinCounterCfg.backgroundRsrcPos = Point(panelX, 215);
  coinCounterCfg.fontId = cfg.coinPanelFontId;
  coinCounterCfg.fontColor = Color(0x1FA4DFFF); //light blue
  coinCounterCfg.incrTimerId = cfg.coinPanelIncrTimerId;
  coinCounterCfg.decrTimerId = cfg.coinPanelDecrTimerId;
  coinCounterCfg.startValue = 0;
  coinCounterCfg.boundaryRect = Rectangle(1365, 230, 346, 120);

  if (SUCCESS != _coinPanel.init(coinCounterCfg)) {
    LOGERR("Error, _coinPanel.init() failed");
    return FAILURE;
  }

  return SUCCESS;
}

void Panel::draw() const {
  _timePanel.draw();
  _healthPanel.draw();
  _healthIndicator.draw();
  _coinPanel.draw();
  _horDelimiter.draw();
  _vertDelimiter.draw();
}

void Panel::decreaseHealthIndicator(int32_t damage) {
  auto cropRectangle = _healthIndicator.getCropRect();
  cropRectangle.w -= damage;
  _healthIndicator.setCropRect(cropRectangle);
}


