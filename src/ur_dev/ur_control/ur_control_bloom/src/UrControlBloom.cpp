//Corresponding header
#include "ur_control_bloom/UrControlBloom.h"

//System headers

//Other libraries headers
#include "utils/Log.h"

//Own components headers
#include "ur_control_bloom/helpers/UrControlBloomInitHelper.h"

UrControlBloom::UrControlBloom(
    const Ros2CommunicatorInterface &communicatorOutInterface)
    : _communicatorInterface(communicatorOutInterface) {

}

ErrorCode UrControlBloom::init(const std::any &cfg) {
  if (ErrorCode::SUCCESS != UrControlBloomInitHelper::init(cfg, *this)) {
    LOGERR("Error, UrControlBloomInitHelper::init() failed");
    return ErrorCode::FAILURE;
  }

  _communicatorInterface.registerNodeCb(_dashboardProvider);
  _communicatorInterface.registerNodeCb(_externalBridge);
  return ErrorCode::SUCCESS;
}

void UrControlBloom::deinit() {
  _communicatorInterface.unregisterNodeCb(_dashboardProvider);
  _communicatorInterface.unregisterNodeCb(_externalBridge);
  _dashboardProvider->deinit();
  _layout.deinit();
}

void UrControlBloom::draw() const {
  _layout.draw();
}

void UrControlBloom::handleEvent(const InputEvent &e) {
  _layout.handleEvent(e);
}

void UrControlBloom::process() {

}

