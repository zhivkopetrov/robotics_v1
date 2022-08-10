//Corresponding header
#include "ur_control_gui/layout/UrControlGuiLayout.h"

//System headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "ur_control_gui/layout/helpers/UrControlGuiLayoutInitHelper.h"
#include "ur_control_gui/layout/helpers/UrControlGuiLayoutInterfaces.h"

ErrorCode UrControlGuiLayout::init(
    const UrControlGuiLayoutConfig &cfg,
    const UrControlGuiLayoutOutInterface &outInterface,
    UrControlGuiLayoutInterface &interface) {
  if (ErrorCode::SUCCESS != UrControlGuiLayoutInitHelper::init(cfg,
          outInterface, *this)) {
    LOGERR("Error, UrControlGuiLayout::init() failed");
    return ErrorCode::FAILURE;
  }

  produceInterface(interface);
  return ErrorCode::SUCCESS;
}

void UrControlGuiLayout::deinit() {

}

void UrControlGuiLayout::draw() const {
  _map.draw();
  _robot.draw();

  _buttonHandler.draw();
  _safetyModeVisuals.draw();
}

void UrControlGuiLayout::handleEvent(const InputEvent &e) {
  _buttonHandler.handleEvent(e);
}

void UrControlGuiLayout::produceInterface(
    UrControlGuiLayoutInterface &interface) {
  using namespace std::placeholders;

  interface.robotModeChangeCb = std::bind(&SafetyModeVisuals::changeRobotMode,
      &_safetyModeVisuals, _1);
  interface.safetyModeChangeCb = std::bind(&SafetyModeVisuals::changeSafetyMode,
      &_safetyModeVisuals, _1);
}

