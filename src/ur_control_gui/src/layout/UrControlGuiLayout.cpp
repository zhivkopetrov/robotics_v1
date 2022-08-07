//Corresponding header
#include "ur_control_gui/layout/UrControlGuiLayout.h"

//System headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "ur_control_gui/layout/helpers/UrControlGuiLayoutInitHelper.h"

ErrorCode UrControlGuiLayout::init(
    const UrControlGuiLayoutConfig &cfg,
    const UrControlGuiLayoutOutInterface &outInterface) {
  if (ErrorCode::SUCCESS != UrControlGuiLayoutInitHelper::init(cfg,
          outInterface, *this)) {
    LOGERR("Error, UrControlGuiLayout::init() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

void UrControlGuiLayout::deinit() {

}

void UrControlGuiLayout::draw() const {
  _map.draw();
  _robot.draw();

  _buttonHandler.draw();
}

void UrControlGuiLayout::handleEvent(const InputEvent &e) {
  _buttonHandler.handleEvent(e);
}

