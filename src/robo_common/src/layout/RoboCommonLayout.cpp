//Corresponding header
#include "robo_common/layout/RoboCommonLayout.h"

//System headers

//Other libraries headers
#include "utils/input/InputEvent.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_common/layout/helpers/RoboCommonLayoutInitHelper.h"

ErrorCode RoboCommonLayout::init(
    const RoboCommonLayoutConfig &cfg,
    const RoboCommonLayoutOutInterface &outInterface,
    RoboCommonLayoutInterface &interface) {
  if (ErrorCode::SUCCESS != RoboCommonLayoutInitHelper::init(cfg, outInterface,
          *this)) {
    LOGERR("Error, RoboCommonLayoutInitHelper::init() failed");
    return ErrorCode::FAILURE;
  }

  interface = produceInterface();
  return ErrorCode::SUCCESS;
}

void RoboCommonLayout::deinit() {
  _playerRobot.deinit();
}

void RoboCommonLayout::draw() const {
  _map.draw();
  _field.draw();
  _playerRobot.draw();
}

void RoboCommonLayout::handleEvent(const InputEvent &e) {
  if (TouchEvent::KEYBOARD_RELEASE == e.type) {
    if (Keyboard::KEY_F == e.key) {
      _fogOfWar.revealAllFogTiles();
    }
  }
}

void RoboCommonLayout::drawSecondLayer() const {
  _fogOfWar.draw();
  _debugField.draw();
  _helpPageAnimator.draw();
}

void RoboCommonLayout::drawThirdLayer() const {
  _gameEndAnimator.draw();
  _achievementAnimator.draw(); //is drawn as part of end screen animator
}

void RoboCommonLayout::process() {
  _debugField.process();
}

void RoboCommonLayout::toggleHelpPage() {
  _helpPageAnimator.toggleStatus();
}

void RoboCommonLayout::toggleDebugInfo() {
  _field.toggleDebugTexts();
  _debugField.toggleStatus();
}

RoboCommonLayoutInterface RoboCommonLayout::produceInterface() {
  using namespace std::placeholders;

  RoboCommonLayoutInterface interface;
  interface.setFieldDataMarkerCb = std::bind(&Field::setFieldDataMarker,
      &_field, _1, _2);
  interface.resetFieldDataMarkerCb = std::bind(&Field::resetFieldDataMarker,
      &_field, _1);
  interface.getFieldDescriptionCb = std::bind(&Field::getDescription, &_field);
  interface.getPlayerSurroundingTilesCb = std::bind(&Robot::getSurroundingTiles,
      &_playerRobot);
  interface.playerRobotActInterface = { std::bind(&Robot::act, &_playerRobot,
      _1), std::bind(&Robot::getState, &_playerRobot), std::bind(
      &Robot::cancelMove, &_playerRobot), std::bind(&Robot::getAbsolutePos,
      &_playerRobot), std::bind(&Robot::getRotationAngle, &_playerRobot) };
  interface.toggleHelpPageCb = std::bind(&RoboCommonLayout::toggleHelpPage,
      this);
  interface.toggleDebugInfoCb = std::bind(&RoboCommonLayout::toggleDebugInfo,
      this);
  interface.setDebugMsgCb = std::bind(&DebugField::setMsg, &_debugField, _1);
  interface.startGameWonAnimCb = std::bind(&GameEndAnimator::startGameWonAnim,
      &_gameEndAnimator);
  interface.startGameLostAnimCb = std::bind(&GameEndAnimator::startGameLostAnim,
      &_gameEndAnimator);
  interface.setUserDataCb = std::bind(&GameEndAnimator::setUserData,
      &_gameEndAnimator, _1);
  interface.startAchievementWonAnimCb = std::bind(
      &GameEndAnimator::startAchievementWonAnim, &_gameEndAnimator, _1);
  interface.revealFogOfWarTilesCb = std::bind(&FogOfWar::revealAllFogTiles,
      &_fogOfWar);

  return interface;
}

