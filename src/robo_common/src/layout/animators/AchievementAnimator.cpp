//Corresponding header
#include "robo_common/layout/animators/AchievementAnimator.h"

//System headers

//Other libraries headers
#include "utils/data_type/EnumClassUtils.h"
#include "utils/drawing/WidgetAligner.h"
#include "utils/Log.h"

//Own components headers

namespace {
constexpr int32_t SINGLE_SCALED_WIDTH = 131;
constexpr int32_t SINGLE_SCALED_HEIGHT = 125;

constexpr int32_t DOUBLE_START_OFFSET_Y = 38;
constexpr int32_t ALL_STARS_OFFSET_X_PER_FRAME = 114;
}

ErrorCode AchievementAnimator::init(
    const AchievementAnimatorConfig &cfg,
    const OnAchievementWonAnimFinished &onAchievementWonAnimFinished) {
  if (nullptr == onAchievementWonAnimFinished) {
    LOGERR("Error, nullptr provided for OnAchievementWonAnimFinished");
    return ErrorCode::FAILURE;
  }
  _onAchievementWonAnimFinished = onAchievementWonAnimFinished;

  _animData.timerId = cfg.fadeAndMoveTimerId;
  _animData.currOpacity = ZERO_OPACITY;

  _allStars.create(cfg.allStarsRsrcId);
  _allStars.activateAlphaModulation();
  _allStars.setOpacity(_animData.currOpacity);
  const Rectangle allStarsFrameRect = _allStars.getFrameRect();
  const Point allStarsPos = WidgetAligner::getPosition(allStarsFrameRect.w,
      allStarsFrameRect.h, cfg.screenDimensions,
      WidgetAlignment::CENTER_CENTER);
  _allStars.setPosition(allStarsPos);
  _allStars.hide();

  _animData.allStarsDimensions = Rectangle(allStarsPos, allStarsFrameRect.w,
      allStarsFrameRect.h);
  _animData.allStarsDimensions.x = allStarsPos.x;
  _animData.allStarsDimensions.y = allStarsPos.y;

  _singleStar.create(cfg.singleStarRsrcId);
  _singleStar.setFrame(1); //yellow star
  _singleStar.activateScaling();
  _singleStar.activateAlphaModulation();
  _singleStar.setOpacity(_animData.currOpacity);
  _singleStar.setScaledWidth(SINGLE_SCALED_WIDTH);
  _singleStar.setScaledHeight(SINGLE_SCALED_HEIGHT);
  _singleStar.setPosition(allStarsPos);
  _singleStar.hide();

  return ErrorCode::SUCCESS;
}

void AchievementAnimator::draw() const {
  _allStars.draw();
  _singleStar.draw();
}

void AchievementAnimator::startAnim(Achievement achievement) {
  _isActive = true;
  _allStars.show();
  _achievementsToProcess.push(achievement);
  if (1 < _achievementsToProcess.size()) {
    return; //currently process another achievement
  }

  configureAnim(_achievementsToProcess.front());

  constexpr int32_t interval = 50;
  startTimer(interval, _animData.timerId, TimerType::PULSE);
}

void AchievementAnimator::startEndGameSequence(
    const std::set<Achievement> &achievements) {
  _endGameSequenceStarted = true;
  _allStars.setFrame(0); //grayed out stars
  _allStars.setOpacity(FULL_OPACITY);
  for (const Achievement ach : achievements) {
    startAnim(ach);
  }
}

bool AchievementAnimator::isActive() const {
  return _isActive;
}

void AchievementAnimator::onTimeout(const int32_t timerId) {
  if (timerId == _animData.timerId) {
    processAnim();
  } else {
    LOGERR("Error, received unsupported timerId: %d", timerId);
  }
}

void AchievementAnimator::processAnim() {
  if (AnimDir::FORWARD == _animData.dir) {
    ++_animData.currStepIdx;
    Point pos = _singleStar.getPosition();
    pos += _animData.moveOffset;
    _singleStar.setPosition(pos);
    _animData.currOpacity += _animData.opacityStep;
    _singleStar.setOpacity(_animData.currOpacity);

    if (!_endGameSequenceStarted) {
      _allStars.setOpacity(_animData.currOpacity);
    }

    if (_animData.lastStepIdx == _animData.currStepIdx) {
      _animData.currStepIdx = 0;

      //+1, because SINGLE_STAR is 0, while the image frame is 1
      const int32_t frameIdx = getEnumValue(_achievementsToProcess.front()) + 1;
      _allStars.setFrame(frameIdx);

      //don't uses AnimDir::BACKWARD when end sequence is requested
      if (_endGameSequenceStarted) {
        _animData.currOpacity = ZERO_OPACITY;
        _singleStar.setOpacity(_animData.currOpacity);
        consumeProcessedAchievement();
      } else {
        _animData.dir = AnimDir::BACKWARD;
        _singleStar.hide();
      }
    }
  } else {
    ++_animData.currStepIdx;
    _animData.currOpacity -= _animData.opacityStep;
    _singleStar.setOpacity(_animData.currOpacity);
    _allStars.setOpacity(_animData.currOpacity);

    if (_animData.lastStepIdx == _animData.currStepIdx) {
      _animData.dir = AnimDir::FORWARD;
      _animData.currStepIdx = 0;

      consumeProcessedAchievement();
    }
  }
}

void AchievementAnimator::configureAnim(Achievement achievement) {
  _singleStar.show();
  _singleStar.setPosition(_animData.allStarsDimensions.x,
      _animData.allStarsDimensions.y);

  const int32_t offset = _animData.totalSteps * _animData.moveStep;
  _singleStar.moveUp(offset);
  _animData.moveOffset.y = _animData.moveStep;
  switch (achievement) {
  case Achievement::SINGLE_STAR:
    _singleStar.moveLeft(offset);
    _animData.moveOffset.x = _animData.moveStep;
    break;
  case Achievement::DOUBLE_STAR:
    _singleStar.moveDown(DOUBLE_START_OFFSET_Y);
    _singleStar.moveRight(offset + ALL_STARS_OFFSET_X_PER_FRAME);
    _animData.moveOffset.x = -1 * _animData.moveStep;
    break;
  case Achievement::TRIPLE_STAR:
    _singleStar.moveLeft(offset - (2 * ALL_STARS_OFFSET_X_PER_FRAME));
    _animData.moveOffset.x = _animData.moveStep;
    break;
  default:
    LOGERR("Error, received unsupported Achievement value: %d",
        getEnumValue(achievement));
    break;
  }
}

void AchievementAnimator::consumeProcessedAchievement() {
  const Achievement processedAchievement = _achievementsToProcess.front();
  _achievementsToProcess.pop();
  if (_achievementsToProcess.empty()) {
    stopTimer(_animData.timerId);
    _isActive = false;
  } else {
    configureAnim(_achievementsToProcess.front());
  }

  _onAchievementWonAnimFinished(processedAchievement);
}
