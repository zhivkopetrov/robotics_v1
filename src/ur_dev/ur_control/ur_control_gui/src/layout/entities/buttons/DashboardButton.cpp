//Corresponding header
#include "ur_control_gui/layout/entities/buttons/DashboardButton.h"

//System headers

//Other libraries headers
#include "sdl_utils/input/InputEvent.h"
#include "utils/Log.h"

//Own components headers

ErrorCode DashboardButton::init(const DashboardButtonConfig &cfg,
                                const InvokeDashboardCb &invokeDashboardCb) {
  if (nullptr == invokeDashboardCb) {
    LOGERR("Error, nullptr provided for InvokeDashboardCb");
    return ErrorCode::FAILURE;
  }
  _invokeDashboardCb = invokeDashboardCb;
  _command = cfg.command;

  if (ErrorCode::SUCCESS != CommandButton::init(cfg.baseCfg)) {
    LOGERR("Error, CommandButton::init() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

void DashboardButton::handleEvent(const InputEvent &e) {
  if (TouchEvent::TOUCH_PRESS == e.type) {
    setFrame(CLICKED);
  } else if (TouchEvent::TOUCH_RELEASE == e.type) {
    setFrame(UNCLICKED);
    _invokeDashboardCb(_command);
  }
}
