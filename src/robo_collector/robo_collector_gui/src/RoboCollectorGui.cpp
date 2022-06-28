//Corresponding header
#include "robo_collector_gui/RoboCollectorGui.h"

//System headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/helpers/RoboCollectorGuiInitHelper.h"

RoboCollectorGui::RoboCollectorGui(
    const Ros2CommunicatorInterface &communicatorOutInterface)
    : _communicatorInterface(communicatorOutInterface) {

}

ErrorCode RoboCollectorGui::init(const std::any &cfg) {
  if (ErrorCode::SUCCESS != RoboCollectorGuiInitHelper::init(cfg, *this)) {
    LOGERR("Error, RoboCollectorGuiInitHelper::init() failed");
    return ErrorCode::FAILURE;
  }

  _communicatorInterface.registerNodeCb(_controllerExternalBridge);
  return ErrorCode::SUCCESS;
}

void RoboCollectorGui::deinit() {
  _communicatorInterface.unregisterNodeCb(_controllerExternalBridge);
  _layout.deinit();
}

void RoboCollectorGui::draw() const {
  _layout.draw();
}

void RoboCollectorGui::handleEvent(const InputEvent &e) {
  _layout.handleEvent(e);
}

void RoboCollectorGui::process() {
  _layout.process();
  _collisionWatcher.process();
}

