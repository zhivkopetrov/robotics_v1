//Corresponding header
#include "robo_common/layout/animators/GameEndAnimator.h"

//System headers

//Other libraries headers
#include "utils/drawing/WidgetAligner.h"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/log/Log.h"

//Own components headers

ErrorCode GameEndAnimator::init(
    const GameEndAnimatorConfig &cfg,
    const GameEndAnimatorOutInterface &outInterface) {
  if (ErrorCode::SUCCESS != initOutInterface(outInterface)) {
    LOGERR("Error, initOutInterface() failed");
    return ErrorCode::FAILURE;
  }

  createFinalScreenFbo(cfg);

  const OnAppearAnimFinish animCb = std::bind(
      &GameEndAnimator::onAppearAnimFinish, this);
  if (ErrorCode::SUCCESS != _appearAnimator.init(cfg, animCb,
          &_finalScreenFbo)) {
    LOGERR("Error, _appearAnimator.init");
    return ErrorCode::FAILURE;
  }

  const CountdownAnimatorConfig countCfg = { .projectName = cfg.projectName,
      .containerDimensions = _finalScreenFbo.getScaledRect(), .countdownFontId =
          cfg.countdownFontId, .screenshotFontId = cfg.userDataFontId,
      .countdownSeconds = 10, .timerId = cfg.countdownAnimTimerId };
  if (ErrorCode::SUCCESS != _countdownAnimator.init(countCfg,
          outInterface.shutdownGameCb, outInterface.takeScreenshotCb)) {
    LOGERR("Error, _countdownAnimator.init");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

void GameEndAnimator::draw() const {
  if (!_isActive) {
    return;
  }

  _appearAnimator.draw();
  _finalScreenFbo.draw();
  _countdownAnimator.draw();
}

void GameEndAnimator::startGameWonAnim() {
  LOGG("You've Won");
  if (_outInterface.isAchievementAnimatorActive()) {
    _hasPendingEndStatus = true;
    _pendingGameOutcome = EndGameOutcome::WIN;
    return;
  }

  _isActive = true;
  _appearAnimator.startAnim(EndGameOutcome::WIN);
}

void GameEndAnimator::startGameLostAnim() {
  LOGR("You've Lost");
  if (_outInterface.isAchievementAnimatorActive()) {
    _hasPendingEndStatus = true;
    _pendingGameOutcome = EndGameOutcome::LOSE;
    return;
  }

  _isActive = true;
  _appearAnimator.startAnim(EndGameOutcome::LOSE);
}

void GameEndAnimator::startAchievementWonAnim(Achievement achievement) {
  const auto [_, inserted] = _wonAchievements.insert(achievement);
  if (!inserted) {
    LOGERR("Error, StartAchievementWonAnimCb a second time for Achievement: %d",
        getEnumValue(achievement));
    return;
  }

  _outInterface.startAchievementWonAnimCb(achievement);
}

void GameEndAnimator::setUserData(const UserData &userData) {
  _appearAnimator.setUserData(userData);
  _countdownAnimator.setUserName(userData.user);
}

void GameEndAnimator::onAchievementWonAnimFinish(Achievement achievement) {
  if (_hasPendingEndStatus) {
    if (_outInterface.isAchievementAnimatorActive()) {
      return;
    }

    _hasPendingEndStatus = false;
    _isActive = true;
    _appearAnimator.startAnim(_pendingGameOutcome);
    return;
  }

  if (!_isActive) {
    return; //normal achievement during mid-game
  }

  _wonAchievements.erase(achievement);
  if (_wonAchievements.empty()) {
    _countdownAnimator.startAnim();
  }
}

ErrorCode GameEndAnimator::initOutInterface(
    const GameEndAnimatorOutInterface &outInterface) {
  _outInterface = outInterface;
  if (nullptr == _outInterface.shutdownGameCb) {
    LOGERR("Error, nullptr provided for ShutdownGameCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.startAchievementWonAnimCb) {
    LOGERR("Error, nullptr provided for StartAchievementWonAnimCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.isAchievementAnimatorActive) {
    LOGERR("Error, nullptr provided for IsAchievementAnimatorActive");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.startEndGameSequence) {
    LOGERR("Error, nullptr provided for StartEndGameSequence");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.takeScreenshotCb) {
    LOGERR("Error, nullptr provided for TakeScreenshotCb");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

void GameEndAnimator::onAppearAnimFinish() {
  if (!_wonAchievements.empty()) {
    //animate won achievements as part of end game animation
    _outInterface.startEndGameSequence(_wonAchievements);
  } else {
    //no achievements to animate -> skip a phase and jump to next one
    _countdownAnimator.startAnim();
  }
}

void GameEndAnimator::createFinalScreenFbo(const GameEndAnimatorConfig &cfg) {
  const int32_t bgrWidth = static_cast<int32_t>(cfg.endScreenToBgrRatio
      * cfg.screenDimensions.w);
  const int32_t bgrHeight = static_cast<int32_t>(cfg.endScreenToBgrRatio
      * cfg.screenDimensions.h);
  const Point bgrPos = WidgetAligner::getPosition(bgrWidth, bgrHeight,
      cfg.screenDimensions, WidgetAlignment::CENTER_CENTER);
  _finalScreenFbo.create(Rectangle(bgrPos, bgrWidth, bgrHeight));
  _finalScreenFbo.activateScaling();
  _finalScreenFbo.activateAlphaModulation();
  _finalScreenFbo.setResetColor(Colors::FULL_TRANSPARENT);
}

