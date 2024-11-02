//Corresponding header
#include "robo_common/layout/panels/TimePanel.h"

//System headers
#include <string>

//Other libraries headers
#include "utils/drawing/WidgetAligner.h"
#include "utils/log/Log.h"

//Own components headers

ErrorCode TimePanel::init(const TimePanelConfig &cfg,
                          const TimePanelUtilityConfig &utilityCfg) {
  if (nullptr == utilityCfg.timeFinishedCb) {
    LOGERR("Error, nullptr provided for IndicatorDepletedCb");
    return ErrorCode::FAILURE;
  }
  _timeFinishedCb = utilityCfg.timeFinishedCb;

  _clockTimerId = cfg.clockTimerId;
  _blinkTimerId = cfg.blinkTimerId;
  _remainingSeconds = cfg.totalSeconds;

  _panel.create(cfg.rsrcId);
  _panel.setPosition(utilityCfg.pos);

  constexpr auto textOffsetFromPanelX = 146;
  constexpr auto textOffsetFromPanelY = 12;
  constexpr auto textBoundaryWidth = 340;
  constexpr auto textBoundaryHeight = 130;

  const auto startColor = getStartColor(cfg.totalSeconds);
  _timeText.create(cfg.fontId, "0", startColor);
  _textBoundRect = Rectangle(utilityCfg.pos.x + textOffsetFromPanelX,
      utilityCfg.pos.y + textOffsetFromPanelY, textBoundaryWidth,
      textBoundaryHeight);
  setTextContent();

  constexpr auto timerInterval = 1000;
  startTimer(timerInterval, _clockTimerId, TimerType::PULSE);

  return ErrorCode::SUCCESS;
}

void TimePanel::draw() const {
  _panel.draw();
  _timeText.draw();
}

void TimePanel::onTimeout(const int32_t timerId) {
  if (timerId == _clockTimerId) {
    processClockTick();
  } else if (timerId == _blinkTimerId) {
    _timeText.isVisible() ? _timeText.hide() : _timeText.show();
  } else {
    LOGERR("Error, received unsupported timerId: %d", timerId);
  }
}

void TimePanel::processClockTick() {
  --_remainingSeconds;
  setTextContent();

  if (0 == _remainingSeconds) {
    stopTimer(_clockTimerId);
    if (isActiveTimerId(_blinkTimerId)) {
      stopTimer(_blinkTimerId);
    }
    _timeFinishedCb();
    return;
  }

  if (90 == _remainingSeconds) {
    _timeText.setColor(Colors::ORANGE);
  } else if (30 == _remainingSeconds) {
    _timeText.setColor(Colors::RED);
    constexpr auto timerInterval = 250;
    startTimer(timerInterval, _blinkTimerId, TimerType::PULSE);
  }
}

void TimePanel::setTextContent() {
  constexpr auto secondsInMinute = 60;
  const auto minutes = _remainingSeconds / secondsInMinute;
  const auto seconds = _remainingSeconds % secondsInMinute;

  std::string content = std::to_string(minutes);
  content.append(" : ");
  if (10 > seconds) {
    content.append("0");
  }
  content.append(std::to_string(seconds));

  _timeText.setText(content.c_str());

  const auto textPos = WidgetAligner::getPosition(_timeText.getImageWidth(),
      _timeText.getImageHeight(), _textBoundRect,
      WidgetAlignment::CENTER_CENTER);
  _timeText.setPosition(textPos);
}

Color TimePanel::getStartColor(int32_t totalSeconds) const {
  if (90 < totalSeconds) {
    return Colors::GREEN;
  }

  if (30 < totalSeconds) {
    return Colors::ORANGE;
  }

  return Colors::RED;
}

