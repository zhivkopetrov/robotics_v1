//Corresponding header
#include "robo_common/layout/animators/EndScreenAppearAnimator.h"

//System headers
#include <algorithm>

//Other libraries headers
#include "utils/drawing/WidgetAligner.h"
#include "utils/Log.h"

//Own components headers

ErrorCode EndScreenAppearAnimator::init(
    const GameEndAnimatorConfig &cfg,
    const OnAppearAnimFinish &onAppearAnimFinish, Fbo *finalScreenFbo) {
  if (nullptr == onAppearAnimFinish) {
    LOGERR("Error, nullptr provided for OnAppearAnimFinish");
    return ErrorCode::FAILURE;
  }
  _onAppearAnimFinish = onAppearAnimFinish;

  _rsrcIdsLocal.bgrRsrcId = cfg.bgrRsrcId;
  _rsrcIdsLocal.winStatusFontId = cfg.winStatusFontId;
  _rsrcIdsLocal.userDataFontId = cfg.userDataFontId;

  _finalScreenFbo = finalScreenFbo;
  _fadeAnimData.timerId = cfg.fadeAnimTimerId;

  _expandAnimData.timerId = cfg.expandAnimTimerId;
  _expandAnimData.expandedDimensions = finalScreenFbo->getScaledRect();
  const Point finalScreenPos = finalScreenFbo->getPosition();
  _expandAnimData.expandedDimensions.x = finalScreenPos.x;
  _expandAnimData.expandedDimensions.y = finalScreenPos.y;

  const int32_t shrinkedWidth = _expandAnimData.expandedDimensions.w / 2;
  const int32_t shrinkedHeight = _expandAnimData.expandedDimensions.h / 2;
  _expandAnimData.widthStep = shrinkedWidth / _expandAnimData.totalSteps;
  _expandAnimData.heighStep = shrinkedHeight / _expandAnimData.totalSteps;

  createBlackBgrFbo(cfg.screenDimensions);

  return ErrorCode::SUCCESS;
}

void EndScreenAppearAnimator::draw() const {
  _blackBgrFbo.draw();
}

void EndScreenAppearAnimator::setUserData(const UserData &userData) {
  _userData = userData;
}

void EndScreenAppearAnimator::startAnim(EndGameOutcome outcome) {
  createBgrImage();
  createUserDataTexts();
  createEndGameOutcomeText(outcome);
  updateFinalScreenFbo(_finalScreenFbo);

  constexpr int64_t interval = 50;
  startTimer(interval, _fadeAnimData.timerId, TimerType::PULSE);
  startTimer(interval, _expandAnimData.timerId, TimerType::PULSE);
}

void EndScreenAppearAnimator::createBlackBgrFbo(const Rectangle &dimensions) {
  _blackBgrFbo.create(dimensions);
  _blackBgrFbo.setResetColor(Colors::BLACK);
  _blackBgrFbo.unlock();
  _blackBgrFbo.reset();
  _blackBgrFbo.lock();
  _blackBgrFbo.activateAlphaModulation();
  _blackBgrFbo.setOpacity(_fadeAnimData.currOpacity);
}

void EndScreenAppearAnimator::createBgrImage() {
  _bgrImg.create(_rsrcIdsLocal.bgrRsrcId);
  _bgrImg.activateScaling();
  _bgrImg.setScaledWidth(_finalScreenFbo->getFrameWidth());
  _bgrImg.setScaledHeight(_finalScreenFbo->getFrameHeight());
  _bgrImg.setPosition(_finalScreenFbo->getPosition());
}

void EndScreenAppearAnimator::createUserDataTexts() {
  constexpr int32_t lastTextIdx = USER_TEXTS_COUNT - 1;
  std::array<std::string, USER_TEXTS_COUNT> textsContent { "User: "
      + _userData.user, "Repository: " + _userData.repository, "Commit SHA: "
      + _userData.commitSha };

  const Rectangle containerBoundary = _finalScreenFbo->getScaledRect();
  constexpr int32_t offset = 10;
  const int32_t maxTextWidth = (containerBoundary.w / 2) - (4 * offset);

  std::array<int32_t, USER_TEXTS_COUNT> textWidths;
  for (int32_t i = 0; i < USER_TEXTS_COUNT; ++i) {
    _userTexts[i].create(_rsrcIdsLocal.userDataFontId, textsContent[i].c_str(),
        Colors::RED);
    _userTexts[i].activateScaling();
    _userTexts[i].setMaxScalingWidth(maxTextWidth);
    textWidths[i] = _userTexts[i].getScaledWidth();
  }

  auto it = std::max_element(textWidths.begin(), textWidths.end());
  const size_t maxTextWidthIdx = it - textWidths.begin();

  Point textPos = WidgetAligner::getPosition(
      _userTexts[maxTextWidthIdx].getScaledWidth(),
      _userTexts[lastTextIdx].getScaledHeight(), containerBoundary,
      WidgetAlignment::LOWER_RIGHT, Margin(30, 30, 30, 30));

  int32_t newY { };
  for (int32_t i = lastTextIdx; 0 <= i; --i) {
    newY = textPos.y - i * (_userTexts[i].getFrameHeight() + offset);
    const int32_t currTextIdx = lastTextIdx - i;
    _userTexts[currTextIdx].setPosition(textPos.x, newY);
  }
}

void EndScreenAppearAnimator::createEndGameOutcomeText(EndGameOutcome outcome) {
  const std::string textStr =
      (EndGameOutcome::WIN == outcome) ? "You WIN" : "You Lose";
  _endGameOutcomeText.create(_rsrcIdsLocal.winStatusFontId, textStr.c_str(),
      Colors::RED);

  const Rectangle frameRect = _endGameOutcomeText.getFrameRect();
  const Point pos = WidgetAligner::getPosition(frameRect.w, frameRect.h,
      _expandAnimData.expandedDimensions, WidgetAlignment::UPPER_CENTER,
      Margin(50, 0, 0, 0));
  _endGameOutcomeText.setPosition(pos);
}

void EndScreenAppearAnimator::onTimeout(const int32_t timerId) {
  if (timerId == _expandAnimData.timerId) {
    processExpandAnim();
  } else if (timerId == _fadeAnimData.timerId) {
    processFadeAnim();
  } else {
    LOGERR("Error, received unsupported timerId: %d", timerId);
  }
}

void EndScreenAppearAnimator::processFadeAnim() {
  _fadeAnimData.currOpacity += _fadeAnimData.fadeStep;
  _blackBgrFbo.setOpacity(_fadeAnimData.currOpacity);
  if (_fadeAnimData.endOpacity <= _fadeAnimData.currOpacity) {
    stopTimer(_fadeAnimData.timerId);
  }
}

void EndScreenAppearAnimator::processExpandAnim() {
  ++_expandAnimData.currStepIdx;
  const int32_t width = _finalScreenFbo->getScaledWidth()
      + _expandAnimData.widthStep;
  const int32_t height = _finalScreenFbo->getScaledHeight()
      + _expandAnimData.heighStep;
  _finalScreenFbo->setScaledWidth(width);
  _finalScreenFbo->setScaledHeight(height);

  const Point pos = WidgetAligner::getPosition(width, height,
      _expandAnimData.expandedDimensions, WidgetAlignment::CENTER_CENTER);
  _finalScreenFbo->setPosition(pos);

  if (_expandAnimData.currStepIdx == _expandAnimData.lastStepIdx) {
    stopTimer(_expandAnimData.timerId);
    _onAppearAnimFinish();
  }
}

void EndScreenAppearAnimator::updateFinalScreenFbo(Fbo *finalScreenFbo) {
  finalScreenFbo->unlock();
  finalScreenFbo->reset();

  finalScreenFbo->addWidget(_bgrImg);
  finalScreenFbo->addWidget(_endGameOutcomeText);
  for (const Text &text : _userTexts) {
    finalScreenFbo->addWidget(text);
  }

  finalScreenFbo->update();
  finalScreenFbo->lock();
}

