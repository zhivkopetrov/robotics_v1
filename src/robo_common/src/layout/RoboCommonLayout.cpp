//Corresponding header
#include "robo_common/layout/RoboCommonLayout.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_common/layout/helpers/RoboCommonLayoutInitHelper.h"

int32_t RoboCommonLayout::init(
    const RoboCommonLayoutConfig &cfg,
    const RoboCommonLayoutOutInterface &outInterface,
    RoboCommonLayoutInterface &interface) {
  if (SUCCESS != RoboCommonLayoutInitHelper::init(cfg, outInterface, *this)) {
    LOGERR("Error, RoboCommonLayoutInitHelper::init() failed");
    return FAILURE;
  }

  interface = produceInterface();
  return SUCCESS;
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

void RoboCommonLayout::activateHelpPage() {
  _field.toggleDebugTexts();
}

RoboCommonLayoutInterface RoboCommonLayout::produceInterface() {
  using namespace std::placeholders;

  RoboCommonLayoutInterface interface;
  interface.setFieldDataMarkerCb =
      std::bind(&Field::setFieldDataMarker, &_field, _1, _2);
  interface.resetFieldDataMarkerCb =
      std::bind(&Field::resetFieldDataMarker, &_field, _1);
  interface.getFieldDataCb = std::bind(&Field::getFieldData, &_field);
  interface.playerRobotActInterface = {
      std::bind(&Robot::act, &_playerRobot,_1),
      std::bind(&Robot::getFieldPos, &_playerRobot),
      std::bind(&Robot::getDirection, &_playerRobot) };
  interface.startGameWonAnimCb =
      std::bind(&GameEndAnimator::startGameWonAnim, &_gameEndAnimator);
  interface.startGameLostAnimCb =
      std::bind(&GameEndAnimator::startGameLostAnim, &_gameEndAnimator);

  return interface;
}

