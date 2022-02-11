//Corresponding header
#include "robo_collector_gui/panels/HealthPanel.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/drawing/WidgetAligner.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t HealthPanel::init(const HealthPanelConfig &cfg,
                          const GameLostCb &gameLostCb) {
  _indicatorReduceTimerId = cfg.indicatorReduceTimerId;

  if (nullptr == gameLostCb) {
    LOGERR("Error, nullptr provided for GameLostCb");
    return FAILURE;
  }
  _gameLostCb = gameLostCb;

  constexpr auto panelX = 1250;
  constexpr auto panelY = 390;
  _panel.create(cfg.rsrcId);
  _panel.setPosition(panelX, panelY);

  _indicator.create(cfg.indicatorRsrcId);
  _indicator.setPosition(panelX + 79, panelY + 13);
  _indicator.setCropRect(_indicator.getImageRect());

  _indicatorText.create(cfg.indicatorFontId, "100%", Colors::RED);
  setAndCenterIndicatorText();

  return SUCCESS;
}

void HealthPanel::draw() const {
  _panel.draw();
  _indicator.draw();
  _indicatorText.draw();
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
  auto &remainingHealth = cropRectangle.w;
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
  setAndCenterIndicatorText();
}

void HealthPanel::setAndCenterIndicatorText() {
  const auto totalHealth = _indicator.getImageWidth();
  const auto indicatorCropRect = _indicator.getCropRect();
  const auto remainingHealth = indicatorCropRect.w;
  const auto remainingHealthPecent = (remainingHealth * 100) / totalHealth;
  const auto healthContent = std::to_string(remainingHealthPecent) + "%";

  _indicatorText.setText(healthContent.c_str());

  auto widgetAlignArea = indicatorCropRect;
  if (0 == remainingHealthPecent) {
    widgetAlignArea = _indicator.getImageRect();
  } else if (remainingHealthPecent < 50) {
    widgetAlignArea.x += widgetAlignArea.w;
    widgetAlignArea.w = totalHealth - remainingHealth;
    _indicatorText.setColor(Colors::GREEN);
  }

  const auto indicatorTextWidth = _indicatorText.getImageWidth();
  const auto textPos = WidgetAligner::getPosition(
      indicatorTextWidth, _indicatorText.getImageHeight(),
      widgetAlignArea, WidgetAlignment::CENTER_CENTER);
  _indicatorText.setPosition(textPos);
}
