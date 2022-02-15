//Corresponding header
#include "robo_collector_gui/layout/RoboCollectorLayout.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "robo_collector_gui/layout/helpers/RoboCollectorLayoutInitHelper.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t RoboCollectorLayout::init(
    const RoboCollectorLayoutConfig &cfg,
    const RoboCollectorLayoutOutInterface &outInterface,
    RoboCollectorLayoutInterface& interface) {
  if (SUCCESS != RoboCollectorLayoutInitHelper::init(
      cfg, outInterface, interface.commonLayoutInterface, *this)) {
    LOGERR("Error, RoboCollectorLayoutInitHelper::init() failed");
    return FAILURE;
  }

  produceInterface(interface);
  return SUCCESS;
}

void RoboCollectorLayout::produceInterface(
    RoboCollectorLayoutInterface& interface) {
  using namespace std::placeholders;

  interface.enablePlayerInputCb = std::bind(
      &RoboCollectorController::unlockInput, &_controller);
  interface.moveButtonClickCb = std::bind(
      &RoboCollectorController::onMoveButtonClicked, &_controller, _1);
  for (auto i = 0; i < Defines::ENEMIES_CTN; ++i) {
    interface.enemyRobotActInterfaces.emplace_back(
        std::bind(&Robot::act, &_enemyRobots[i], _1),
        std::bind(&Robot::getFieldPos, &_enemyRobots[i]),
        std::bind(&Robot::getDirection, &_enemyRobots[i]));
  }
}

void RoboCollectorLayout::deinit() {
  for (auto &robot : _enemyRobots) {
    robot.deinit();
  }
  _coinHandler.deinit();
}

void RoboCollectorLayout::draw() const {
  if (GameType::MINER == _gameType) {
    _panelHandler.draw();
    _controller.draw();
    _roboMinerGui.draw();
    return;
  }

  if (GameType::CLEANER == _gameType) {
    _panelHandler.draw();
    _controller.draw();
    _roboCleanerGui.draw();
    return;
  }

  _commonLayout.draw();
  _panelHandler.draw();
  _coinHandler.draw();
  _controller.draw();
  for (const auto &robot : _enemyRobots) {
    robot.draw();
  }
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
  _commonLayout.activateHelpPage();
}

void RoboCollectorLayout::changeGameType(GameType gameType) {
  _gameType = gameType;

  if (GameType::COLLECTOR == _gameType) {
    _controller.unlockInput();
  } else {
    _controller.lockInput();
  }
}

