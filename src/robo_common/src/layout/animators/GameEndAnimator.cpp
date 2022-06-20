//Corresponding header
#include "robo_common/layout/animators/GameEndAnimator.h"

//System headers

//Other libraries headers
#include "utils/drawing/WidgetAligner.h"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/Log.h"

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
  return ErrorCode::SUCCESS;
}

void GameEndAnimator::draw() const {
  if (!_isActive) {
    return;
  }

  _appearAnimator.draw();
  _finalScreenFbo.draw();
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
    LOGC("GameEndAnimator::startGameLostAnim() - PENDING animations");
    _hasPendingEndStatus = true;
    _pendingGameOutcome = EndGameOutcome::LOSE;
    return;
  }

  LOGC("GameEndAnimator::startGameLostAnim() - not pending animations");

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
}

void GameEndAnimator::onAchievementWonAnimFinish(Achievement achievement) {
  LOGC("GameEndAnimator::onAchievementWonAnimFinish()");
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
    LOGC("GameEndAnimator::shutdownGameCb()");
    _outInterface.shutdownGameCb();
    //TODO implement a CountDown class which will be activated if you've won
    //the class will not be part of the fbo, because it will change frequently
    //Text "Please take a screenshot"
    //Application shutting down in "9, 8, 7, 6, 5"

    //TODO automatically save screenshot
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

  return ErrorCode::SUCCESS;
}

void GameEndAnimator::onAppearAnimFinish() {
  LOGC("GameEndAnimator::onAppearAnimFinish()");
  _outInterface.startEndGameSequence(_wonAchievements);
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

