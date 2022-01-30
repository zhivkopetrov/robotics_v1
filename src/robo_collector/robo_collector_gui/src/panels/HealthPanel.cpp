//Corresponding header
#include "robo_collector_gui/panels/HealthPanel.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t HealthPanel::init(const HealthPanelConfig &cfg,
                          const GameLostCb &gameLostCb) {
  if (nullptr == gameLostCb) {
    LOGERR("Error, nullptr provided for GameLostCb");
    return FAILURE;
  }
  _gameLostCb = gameLostCb;

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
  _damageTicksLeft += damage;
  if (!isActiveTimerId(_indicatorReduceTimerId)) {
    constexpr auto timerInterval = 20;
    startTimer(timerInterval, _indicatorReduceTimerId, TimerType::PULSE);
  }
}

void HealthPanel::onTimeout(const int32_t timerId) {
  if (timerId == _indicatorReduceTimerId) {
    processIndicatorReduceAnim();
  } else {
    LOGERR("Error, received unsupported timerId: %d", timerId);
  }
}

void HealthPanel::processIndicatorReduceAnim() {
  auto cropRectangle = _indicator.getCropRect();
  auto& remainingHealth = cropRectangle.w;
  if (0 == remainingHealth) {
    stopTimer(_indicatorReduceTimerId);
    _gameLostCb();
    return;
  }

  if (0 == _damageTicksLeft) {
    stopTimer(_indicatorReduceTimerId);
    return;
  }
  --_damageTicksLeft;

  --remainingHealth;
  _indicator.setCropRect(cropRectangle);
}
