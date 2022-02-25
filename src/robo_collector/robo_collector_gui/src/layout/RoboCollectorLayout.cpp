//Corresponding header
#include "robo_collector_gui/layout/RoboCollectorLayout.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/layout/helpers/RoboCollectorLayoutInterfaces.h"
#include "robo_collector_gui/layout/helpers/RoboCollectorLayoutInitHelper.h"

using namespace std::placeholders;

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
  interface.enablePlayerInputCb = std::bind(
      &RoboCollectorUiController::unlockInput, &_controller);
  interface.moveButtonClickCb = std::bind(
      &RoboCollectorUiController::onMoveButtonClicked, &_controller, _1);
  for (auto i = 0; i < Defines::ENEMIES_CTN; ++i) {
    interface.enemyRobotActInterfaces.emplace_back(
        std::bind(&Robot::act, &_enemyRobots[i], _1),
        std::bind(&Robot::getState, &_enemyRobots[i]));
  }
}

void RoboCollectorLayout::deinit() {
  _commonLayout.deinit();
  for (auto &robot : _enemyRobots) {
    robot.deinit();
  }
  _coinHandler.deinit();
}

void RoboCollectorLayout::draw() const {
  _commonLayout.draw();
  _panelHandler.draw();
  _coinHandler.draw();
  for (const auto &robot : _enemyRobots) {
    robot.draw();
  }

  if (_controller.isEnabled()) {
    _controller.draw();
  }
}

void RoboCollectorLayout::handleEvent(const InputEvent &e) {
  if (_controller.isEnabled()) {
    _controller.handleEvent(e);
  }
}

void RoboCollectorLayout::activateHelpPage() {
  _commonLayout.activateHelpPage();
}

void RoboCollectorLayout::settingsActivated() {

}

