//Corresponding header
#include "robo_miner_gui/RoboMinerGui.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_miner_gui/helpers/RoboMinerGuiInitHelper.h"

RoboMinerGui::RoboMinerGui(
    const Ros2CommunicatorInterface &communicatorOutInterface)
    : _communicatorInterface(communicatorOutInterface) {

}

int32_t RoboMinerGui::init(const std::any &cfg) {
  if (SUCCESS != RoboMinerGuiInitHelper::init(cfg, *this)) {
    LOGERR("Error, RoboMinerGuiInitHelper::init() failed");
    return FAILURE;
  }

  _communicatorInterface.registerNodeCb(_controllerExternalBridge);
  return SUCCESS;
}

void RoboMinerGui::deinit() {
  _communicatorInterface.unregisterNodeCb(_controllerExternalBridge);
  _layout.deinit();
}

void RoboMinerGui::draw() const {
  _layout.draw();
}

void RoboMinerGui::handleEvent(const InputEvent &e) {
  _layout.handleEvent(e);
}

void RoboMinerGui::process() {
  _collisionWatcher.process();
}

void RoboMinerGui::onRobotTurnFinish(const RobotState& state,
                                     MoveOutcome moveOutcome) {
  _movementWatcher.changeState(state, moveOutcome);
}

