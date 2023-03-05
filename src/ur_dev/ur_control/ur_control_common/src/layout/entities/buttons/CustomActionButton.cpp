//Corresponding header
#include "ur_control_common/layout/entities/buttons/CustomActionButton.h"

//System headers

//Other libraries headers
#include "utils/input/InputEvent.h"
#include "utils/Log.h"

//Own components headers

ErrorCode CustomActionButton::init(
  const CommandButtonConfig &cfg, const CustomActionCb &customActionCb) {
  if (nullptr == customActionCb) {
    LOGERR("Error, nullptr provided for CustomActionCb");
    return ErrorCode::FAILURE;
  }
  _customActionCb = customActionCb;

  if (ErrorCode::SUCCESS != CommandButton::init(cfg)) {
    LOGERR("Error, CommandButton::init() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

void CustomActionButton::handleEvent(const InputEvent &e) {
  if (TouchEvent::TOUCH_PRESS == e.type) {
    setFrame(CLICKED);
  } else if (TouchEvent::TOUCH_RELEASE == e.type) {
    setFrame(UNCLICKED);
    _customActionCb();
  }
}
