//Corresponding header
#include "robo_common/layout/animators/HelpPageAnimator.h"

//System headers
#include <cmath>

//Other libraries headers
#include "manager_utils/drawing/Image.h"
#include "manager_utils/drawing/Text.h"
#include "utils/drawing/WidgetAligner.h"
#include "utils/Log.h"

//Own components headers
#include "robo_common/layout/animators/config/HelpPageAnimatorConfig.h"

ErrorCode HelpPageAnimator::init(const HelpPageAnimatorConfig &cfg) {
  _animData.timerId = cfg.moveAndFadeAnimTimerId;
  createBlackBgrFbo(cfg.screenDimensions);
  createHelpPageFbo(cfg);

  const Rectangle helpPageBoundary = _helpPageFbo.getImageRect();
  const int32_t yMoveDistance = std::abs(helpPageBoundary.y)
      + helpPageBoundary.h;
  _animData.totalDownRemainder = yMoveDistance % _animData.totalSteps;
  _animData.leftRemainderMoveDown = _animData.totalDownRemainder;
  _animData.moveStep = yMoveDistance / _animData.totalSteps;

  //set initial state after the FBO has been populated in it's relative position
  _helpPageFbo.moveUp(yMoveDistance);

  return ErrorCode::SUCCESS;
}

void HelpPageAnimator::toggleStatus() {
  if (isActiveTimerId(_animData.timerId)) {
    changeAnimDirection();
    return;
  }

  _isActive = true;
  constexpr int64_t interval = 20;
  startTimer(interval, _animData.timerId, TimerType::PULSE);
}

void HelpPageAnimator::draw() const {
  if (!_isActive) {
    return;
  }

  _blackBgrFbo.draw();
  _helpPageFbo.draw();
}

void HelpPageAnimator::onTimeout(const int32_t timerId) {
  if (timerId == _animData.timerId) {
    if (AnimType::SHOW == _currAnimType) {
      processShowAnim();
    } else {
      processHideAnim();
    }
  } else {
    LOGERR("Error, received unsupported timerId: %d", timerId);
  }
}

void HelpPageAnimator::createBlackBgrFbo(const Rectangle &dimensions) {
  _blackBgrFbo.create(dimensions);
  _blackBgrFbo.setResetColor(Colors::BLACK);
  _blackBgrFbo.unlock();
  _blackBgrFbo.reset();
  _blackBgrFbo.lock();
  _blackBgrFbo.activateAlphaModulation();
  _blackBgrFbo.setOpacity(_animData.currOpacity);
}

void HelpPageAnimator::createHelpPageFbo(const HelpPageAnimatorConfig &cfg) {
  const int32_t scaledWidth = static_cast<int32_t>(cfg.screenDimensions.w
      * cfg.bgrToScreenRatio);
  const int32_t scaledHeight = static_cast<int32_t>(cfg.screenDimensions.h
      * cfg.bgrToScreenRatio);

  Image bgrImg;
  bgrImg.create(cfg.bgrRsrcId);
  bgrImg.activateScaling();
  bgrImg.setScaledWidth(scaledWidth);
  bgrImg.setScaledHeight(scaledHeight);
  Point pos = WidgetAligner::getPosition(scaledWidth, scaledHeight,
      cfg.screenDimensions, WidgetAlignment::CENTER_CENTER);
  bgrImg.setPosition(pos);

  const Rectangle helpPageBoundary = Rectangle(pos, scaledWidth, scaledHeight);
  _helpPageFbo.create(helpPageBoundary);
  _helpPageFbo.activateAlphaModulation();
  _helpPageFbo.setResetColor(Colors::FULL_TRANSPARENT);
  _helpPageFbo.unlock();
  _helpPageFbo.reset(); //to apply alpha background
  _helpPageFbo.lock();

  _helpPageFbo.addWidget(bgrImg);

  //populate texts
  Text titleText;
  titleText.create(cfg.titleEntry.fontRsrcId, cfg.titleEntry.content.c_str(),
      cfg.titleEntry.color);
  const int32_t titleTextWidth = titleText.getFrameWidth();
  const int32_t titleTextHeight = titleText.getFrameHeight();
  constexpr int32_t offset = 10;
  constexpr int32_t doubleOffset = offset * 2;
  pos = WidgetAligner::getPosition(titleTextWidth, titleTextHeight,
      helpPageBoundary, WidgetAlignment::UPPER_CENTER,
      Margin(doubleOffset, doubleOffset, doubleOffset, doubleOffset));
  titleText.setPosition(pos);
  _helpPageFbo.addWidget(titleText);

  //prepare y position for first entry text
  pos.y += (doubleOffset + titleTextHeight);
  pos.x = helpPageBoundary.x + (offset * 4);

  const size_t entriesSize = cfg.entries.size();
  std::vector<Text> entryTexts(entriesSize);
  for (size_t i = 0; i < entriesSize; ++i) {
    const auto &textCfg = cfg.entries[i];

    //apply vertical spacing (if such is set)
    pos.y += textCfg.prependedVerticalSpacing;

    entryTexts[i].create(textCfg.fontRsrcId, textCfg.content.c_str(),
        textCfg.color, pos);

    _helpPageFbo.addWidget(entryTexts[i]);
    pos.y += (entryTexts[i].getFrameHeight() + offset);
  }

  _helpPageFbo.unlock();
  _helpPageFbo.update();
  _helpPageFbo.lock();
}

void HelpPageAnimator::processShowAnim() {
  if (_animData.lastShowStepIdx == _animData.currStepIdx) {
    stopTimer(_animData.timerId);
    changeAnimDirection();
    return;
  }

  ++_animData.currStepIdx;
  _animData.currOpacity += _animData.opacityStep;
  _blackBgrFbo.setOpacity(_animData.currOpacity);

  _helpPageFbo.moveDown(_animData.moveStep);

  if (0 < _animData.leftRemainderMoveDown) {
    --_animData.leftRemainderMoveDown;
    _helpPageFbo.moveDown(1);
  }
}

void HelpPageAnimator::processHideAnim() {
  if (_animData.lastHideStepIdx == _animData.currStepIdx) {
    _isActive = false;
    stopTimer(_animData.timerId);
    changeAnimDirection();
    return;
  }

  //on end condition
  --_animData.currStepIdx;
  _animData.currOpacity -= _animData.opacityStep;
  _blackBgrFbo.setOpacity(_animData.currOpacity);

  _helpPageFbo.moveUp(_animData.moveStep);

  if (0 > _animData.leftRemainderMoveDown) {
    ++_animData.leftRemainderMoveDown;
    _helpPageFbo.moveUp(1);
  }
}

void HelpPageAnimator::changeAnimDirection() {
  if (AnimType::SHOW == _currAnimType) {
    _currAnimType = AnimType::HIDE;
    _animData.leftRemainderMoveDown = -1 * _animData.totalDownRemainder;
  } else {
    _currAnimType = AnimType::SHOW;
    _animData.leftRemainderMoveDown = _animData.totalDownRemainder;
  }
}
