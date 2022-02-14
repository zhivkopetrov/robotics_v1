//Corresponding header
#include "robo_collector_gui/layout/RoboCollectorLayout.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "robo_collector_gui/layout/RoboCollectorLayoutInitHelper.h"
#include "sdl_utils/input/InputEvent.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t RoboCollectorLayout::init(
    const RoboCollectorLayoutConfig &cfg,
    const RoboCollectorLayoutOutInterface &interface) {
  if (SUCCESS != RoboCollectorLayoutInitHelper::init(cfg, interface, *this)) {
    LOGERR("Error, RoboCollectorLayoutInitHelper::init() failed");
    return FAILURE;
  }

  return SUCCESS;
}

RoboCollectorLayoutInterface RoboCollectorLayout::produceInterface() {
  using namespace std::placeholders;

  RoboCollectorLayoutInterface interface;
  interface.enablePlayerInputCb = std::bind(
      &RoboCollectorController::unlockInput, &_controller);
  interface.getFieldDataCb = std::bind(&Field::getFieldData, &_field);
  interface.moveButtonClickCb = std::bind(
      &RoboCollectorController::onMoveButtonClicked, &_controller, _1);
  for (auto i = 0; i < Defines::ROBOTS_CTN; ++i) {
    interface.robotActInterfaces.emplace_back(
        std::bind(&Robot::act, &_robots[i], _1),
        std::bind(&Robot::getFieldPos, &_robots[i]),
        std::bind(&Robot::getDirection, &_robots[i]));
  }

  return interface;
}

void RoboCollectorLayout::deinit() {
  for (auto &robot : _robots) {
    robot.deinit();
  }
  _coinHandler.deinit();
}

void RoboCollectorLayout::draw() const {
  if (GameType::MINER == _gameType) {
    _map.draw();
    _field.draw();
    _panelHandler.draw();
    _controller.draw();
    _robots[RoboCommonDefines::PLAYER_ROBOT_IDX].draw();
    _roboMinerGui.draw();
    return;
  }

  if (GameType::CLEANER == _gameType) {
    _map.draw();
    _field.draw();
    _panelHandler.draw();
    _controller.draw();
    _robots[RoboCommonDefines::PLAYER_ROBOT_IDX].draw();
    _roboCleanerGui.draw();
    return;
  }

  _map.draw();
  _field.draw();
  _panelHandler.draw();
  _coinHandler.draw();
  _controller.draw();
  for (const auto &robot : _robots) {
    robot.draw();
  }

  _gameEndAnimator.draw();
}

void RoboCollectorLayout::handleEvent(const InputEvent &e) {
  _controller.handleEvent(e);

  if (GameType::MINER == _gameType) {
    _roboMinerGui.handleEvent(e);
    return;
  }

  if (GameType::CLEANER == _gameType) {
    return;
  }
}

void RoboCollectorLayout::activateHelpPage() {
  _field.toggleDebugTexts();
}

void RoboCollectorLayout::changeGameType(GameType gameType) {
  _gameType = gameType;

  if (GameType::COLLECTOR == _gameType) {
    _controller.unlockInput();
  } else {
    _controller.lockInput();
  }
}

