//Corresponding header
#include "robo_miner_gui/RoboMinerGui.h"

//System headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_miner_gui/helpers/RoboMinerGuiInitHelper.h"

RoboMinerGui::RoboMinerGui(
    const Ros2CommunicatorInterface &communicatorOutInterface)
    : _communicatorInterface(communicatorOutInterface) {

}

ErrorCode RoboMinerGui::init(const std::any &cfg) {
  if (ErrorCode::SUCCESS != RoboMinerGuiInitHelper::init(cfg, *this)) {
    LOGERR("Error, RoboMinerGuiInitHelper::init() failed");
    return ErrorCode::FAILURE;
  }

  _communicatorInterface.registerNodeCb(_controllerExternalBridge);
  return ErrorCode::SUCCESS;
}

void RoboMinerGui::deinit() {
  _controllerExternalBridge->deinit();
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
  _layout.process();
  _collisionWatcher.process();
}

