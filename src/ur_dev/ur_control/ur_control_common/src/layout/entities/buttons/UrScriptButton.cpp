//Corresponding header
#include "ur_control_common/layout/entities/buttons/UrScriptButton.h"

//System headers

//Other libraries headers
#include "sdl_utils/input/InputEvent.h"
#include "utils/Log.h"

//Own components headers

ErrorCode UrScriptButton::init(const UrScriptButtonConfig &cfg,
                               const PublishURScriptCb &publishURScriptCb) {
  if (nullptr == publishURScriptCb) {
    LOGERR("Error, nullptr provided for PublishURScriptCb");
    return ErrorCode::FAILURE;
  }
  _publishURScriptCb = publishURScriptCb;
  _commandData = cfg.commandData;

  if (ErrorCode::SUCCESS != CommandButton::init(cfg.baseCfg)) {
    LOGERR("Error, CommandButton::init() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

void UrScriptButton::handleEvent(const InputEvent &e) {
  if (TouchEvent::TOUCH_PRESS == e.type) {
    setFrame(CLICKED);
  } else if (TouchEvent::TOUCH_RELEASE == e.type) {
    setFrame(UNCLICKED);
    _publishURScriptCb(_commandData);
  }
}
