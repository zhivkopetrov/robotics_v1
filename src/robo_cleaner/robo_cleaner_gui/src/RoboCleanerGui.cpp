//Corresponding header
#include "robo_cleaner_gui/RoboCleanerGui.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_cleaner_gui/helpers/RoboCleanerGuiInitHelper.h"

RoboCleanerGui::RoboCleanerGui(
    const Ros2CommunicatorInterface &communicatorOutInterface)
    : _communicatorInterface(communicatorOutInterface) {

}

int32_t RoboCleanerGui::init(const std::any &cfg) {
  if (SUCCESS != RoboCleanerGuiInitHelper::init(cfg, *this)) {
    LOGERR("Error, RoboCleanerGuiInitHelper::init() failed");
    return FAILURE;
  }

  _communicatorInterface.registerNodeCb(_controllerExternalBridge);
  return SUCCESS;
}

void RoboCleanerGui::deinit() {
  _communicatorInterface.unregisterNodeCb(_controllerExternalBridge);
  _layout.deinit();
}

void RoboCleanerGui::draw() const {
  _layout.draw();
}

void RoboCleanerGui::handleEvent([[maybe_unused]]const InputEvent &e) {
}

void RoboCleanerGui::process() {
  _collisionWatcher.process();
}

void RoboCleanerGui::onRobotTurnFinish([[maybe_unused]]int32_t robotId) {

}

