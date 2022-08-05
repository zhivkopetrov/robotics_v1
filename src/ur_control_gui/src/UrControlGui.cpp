//Corresponding header
#include "ur_control_gui/UrControlGui.h"

//System headers

//Other libraries headers
#include "utils/Log.h"

//Own components headers
#include "ur_control_gui/helpers/UrControlGuiInitHelper.h"

UrControlGui::UrControlGui(
    const Ros2CommunicatorInterface &communicatorOutInterface)
    : _communicatorInterface(communicatorOutInterface) {

}

ErrorCode UrControlGui::init(const std::any &cfg) {
  if (ErrorCode::SUCCESS != UrControlGuiInitHelper::init(cfg, *this)) {
    LOGERR("Error, UrControlGuiInitHelper::init() failed");
    return ErrorCode::FAILURE;
  }

  _communicatorInterface.registerNodeCb(_guiExternalBridge);
  return ErrorCode::SUCCESS;
}

void UrControlGui::deinit() {
  _communicatorInterface.unregisterNodeCb(_guiExternalBridge);
  _layout.deinit();
}

void UrControlGui::draw() const {
  _layout.draw();
}

void UrControlGui::handleEvent(const InputEvent &e) {
  _layout.handleEvent(e);
}

void UrControlGui::process() {

}

