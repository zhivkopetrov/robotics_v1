//Corresponding header
#include "robo_collector_controller/RoboCollectorController.h"

//System headers

//Other libraries headers
#include "utils/Log.h"

//Own components headers
#include "robo_collector_controller/helpers/RoboCollectorControllerInitHelper.h"

RoboCollectorController::RoboCollectorController(
    const Ros2CommunicatorInterface &communicatorOutInterface)
    : _communicatorInterface(communicatorOutInterface) {

}

ErrorCode RoboCollectorController::init(const std::any &cfg) {
  if (ErrorCode::SUCCESS !=
      RoboCollectorControllerInitHelper::init(cfg, *this)) {
    LOGERR("Error, RoboCollectorControllerInitHelper::init() failed");
    return ErrorCode::FAILURE;
  }

  _communicatorInterface.registerNodeCb(_controllerExternalBridge);
  return ErrorCode::SUCCESS;
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

