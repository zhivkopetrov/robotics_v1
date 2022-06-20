//Corresponding header
#include "robo_common/layout/animators/CountdownAnimator.h"

//System headers
#include <string>

//Other libraries headers
#include "utils/drawing/WidgetAligner.h"
#include "utils/Log.h"

//Own components headers

ErrorCode CountdownAnimator::init(const CountdownAnimatorConfig &cfg,
                                  const ShutdownGameCb &shutdownGameCb) {
  if (nullptr == shutdownGameCb) {
    LOGERR("Error, nullptr provided for ShutdownGameCb");
    return ErrorCode::FAILURE;
  }
  _shutdownGameCb = shutdownGameCb;

  _timerId = cfg.timerId;
  _countdownSecondsLeft = cfg.countdownSeconds;

  createTexts(cfg);

  return ErrorCode::SUCCESS;
}

void CountdownAnimator::draw() const {
  if (!_isActive) {
    return;
  }

  _descriptionText.draw();
  _countdownText.draw();
}

void CountdownAnimator::startAnim() {
  _isActive = true;
  constexpr int64_t interval = 1000;
  startTimer(interval, _timerId, TimerType::PULSE);
}

void CountdownAnimator::onTimeout(const int32_t timerId) {
  if (timerId == _timerId) {
    processAnim();
  } else {
    LOGERR("Error, received unsupported timerId: %d", timerId);
  }
}

void CountdownAnimator::processAnim() {
  --_countdownSecondsLeft;
  if (0 == _countdownSecondsLeft) {
    //TODO automatically save screenshot

    //TODO add user params to robo collector controller yaml file
    //TODO create a message with user data
    //TODO authenticate user to server from robo collector controller
    //TODO do this for all 3 robo games

    stopTimer(_timerId);
    _shutdownGameCb();
    return;
  }

  _countdownText.setText(std::to_string(_countdownSecondsLeft).c_str());
}

void CountdownAnimator::createTexts(const CountdownAnimatorConfig &cfg) {
  _descriptionText.create(cfg.fontId, "System shutting down in ... ",
      Colors::RED);
  const Rectangle descrTextRect = _descriptionText.getFrameRect();

  Point pos = WidgetAligner::getPosition(descrTextRect.w, descrTextRect.h,
      cfg.containerDimensions, WidgetAlignment::LOWER_LEFT);
  _descriptionText.setPosition(pos);

  constexpr int32_t offsetX = 10;
  pos.x += (descrTextRect.w + offsetX);
  _countdownText.create(cfg.fontId,
      std::to_string(cfg.countdownSeconds).c_str(), Colors::RED, pos);
}
