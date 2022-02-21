//Corresponding header
#include "robo_collector_controller/RoboCollectorController.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_controller/helpers/RoboCollectorControllerInitHelper.h"

RoboCollectorController::RoboCollectorController(
    const Ros2CommunicatorInterface &communicatorOutInterface)
    : _communicatorInterface(communicatorOutInterface) {

}

int32_t RoboCollectorController::init(const std::any &cfg) {
  if (SUCCESS != RoboCollectorControllerInitHelper::init(cfg, *this)) {
    LOGERR("Error, RoboCollectorControllerInitHelper::init() failed");
    return FAILURE;
  }

  _communicatorInterface.registerNodeCb(_controllerExternalBridge);
  return SUCCESS;
}

void RoboCollectorController::deinit() {
  _communicatorInterface.unregisterNodeCb(_controllerExternalBridge);
  _layout.deinit();
}

void RoboCollectorController::draw() const {
  _layout.draw();
}

void RoboCollectorController::handleEvent(const InputEvent &e) {
  _layout.handleEvent(e);
}

void RoboCollectorController::process() {

}

