//Corresponding header
#include "robo_collector_gui/RoboCollectorGui.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/helpers/RoboCollectorGuiInitHelper.h"

RoboCollectorGui::RoboCollectorGui(
    const Ros2CommunicatorInterface &communicatorOutInterface)
    : _communicatorInterface(communicatorOutInterface) {

}

int32_t RoboCollectorGui::init(const std::any &cfg) {
  if (SUCCESS != RoboCollectorGuiInitHelper::init(cfg, *this)) {
    LOGERR("Error, RoboCollectorGuiInitHelper::init() failed");
    return FAILURE;
  }

  _communicatorInterface.registerNodeCb(_controllerExternalBridge);
  return SUCCESS;
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
  _collisionWatcher.process();
}

