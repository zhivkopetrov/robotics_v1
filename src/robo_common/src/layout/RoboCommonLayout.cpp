//Corresponding header
#include "robo_common/layout/RoboCommonLayout.h"

//System headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_common/layout/helpers/RoboCommonLayoutInitHelper.h"

ErrorCode RoboCommonLayout::init(const RoboCommonLayoutConfig &cfg,
                               const RoboCommonLayoutOutInterface &outInterface,
                               RoboCommonLayoutInterface &interface) {
  if (ErrorCode::SUCCESS !=
      RoboCommonLayoutInitHelper::init(cfg, outInterface, *this)) {
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
  _gameEndAnimator.draw();
}

void RoboCommonLayout::drawSecondLayer() const {
  _fogOfWar.draw();
}

void RoboCommonLayout::activateHelpPage() {
  _field.toggleDebugTexts();
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
      _1), std::bind(&Robot::getState, &_playerRobot) };
  interface.startGameWonAnimCb = std::bind(&GameEndAnimator::startGameWonAnim,
      &_gameEndAnimator);
  interface.startGameLostAnimCb = std::bind(&GameEndAnimator::startGameLostAnim,
      &_gameEndAnimator);
  interface.startAchievementWonAnimCb = std::bind(
      &GameEndAnimator::startAchievementWonAnim, &_gameEndAnimator, _1);

  return interface;
}

