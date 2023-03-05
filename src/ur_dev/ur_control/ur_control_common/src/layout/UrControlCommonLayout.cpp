//Corresponding header
#include "ur_control_common/layout/UrControlCommonLayout.h"

//System headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "ur_control_common/layout/helpers/UrControlCommonLayoutInitHelper.h"
#include "ur_control_common/layout/helpers/UrControlCommonLayoutInterfaces.h"

ErrorCode UrControlCommonLayout::init(
    const UrControlCommonLayoutConfig &cfg,
    const UrControlCommonLayoutOutInterface &outInterface,
    UrControlCommonLayoutInterface &interface) {
  if (ErrorCode::SUCCESS != 
      UrControlCommonLayoutInitHelper::init(cfg, outInterface, *this)) {
    LOGERR("Error, UrControlCommonLayout::init() failed");
    return ErrorCode::FAILURE;
  }

  produceInterface(interface);
  return ErrorCode::SUCCESS;
}

void UrControlCommonLayout::deinit() {

}

void UrControlCommonLayout::draw() const {
  _map.draw();
  _robot.draw();

  buttonHandler->draw();
  safetyModeVisuals.draw();
}

void UrControlCommonLayout::handleEvent(const InputEvent &e) {
  buttonHandler->handleEvent(e);
}

void UrControlCommonLayout::produceInterface(
    UrControlCommonLayoutInterface &interface) {
  using namespace std::placeholders;

  interface.robotModeChangeCb = std::bind(&SafetyModeVisuals::changeRobotMode,
      &safetyModeVisuals, _1);
  interface.safetyModeChangeCb = std::bind(&SafetyModeVisuals::changeSafetyMode,
      &safetyModeVisuals, _1);
}

