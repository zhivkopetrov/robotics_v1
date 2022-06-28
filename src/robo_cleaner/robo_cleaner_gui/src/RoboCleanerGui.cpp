//Corresponding header
#include "robo_cleaner_gui/RoboCleanerGui.h"

//System headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_cleaner_gui/helpers/RoboCleanerGuiInitHelper.h"

RoboCleanerGui::RoboCleanerGui(
    const Ros2CommunicatorInterface &communicatorOutInterface)
    : _communicatorInterface(communicatorOutInterface) {

}

ErrorCode RoboCleanerGui::init(const std::any &cfg) {
  if (ErrorCode::SUCCESS != RoboCleanerGuiInitHelper::init(cfg, *this)) {
    LOGERR("Error, RoboCleanerGuiInitHelper::init() failed");
    return ErrorCode::FAILURE;
  }

  _communicatorInterface.registerNodeCb(_controllerExternalBridge);
  return ErrorCode::SUCCESS;
}

void RoboCleanerGui::deinit() {
  _movementReporter.deinit();
  _communicatorInterface.unregisterNodeCb(_controllerExternalBridge);
  _layout.deinit();
}

void RoboCleanerGui::draw() const {
  _layout.draw();
}

void RoboCleanerGui::handleEvent([[maybe_unused]]const InputEvent &e) {
}

void RoboCleanerGui::process() {
  _layout.process();
  _collisionWatcher.process();
  _movementWatcher.process();
}

