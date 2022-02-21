//Corresponding header
#include "robo_collector_common/layout/controller/buttons/HelpButton.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "sdl_utils/input/InputEvent.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t HelpButton::init(const HelpButtonConfig& cfg,
                         const HelpActivatedCb& helpActivatedCb) {
  if (nullptr == helpActivatedCb) {
    LOGERR("Error, nullptr provided for HelpActivatedCb");
    return FAILURE;
  }
  _helpActivatedCb = helpActivatedCb;

  create(cfg.rsrcId);
  setPosition(cfg.pos);

  return SUCCESS;
}

void HelpButton::handleEvent(const InputEvent& e) {
  if (TouchEvent::TOUCH_PRESS == e.type) {
    setFrame(CLICKED);
  } else if (TouchEvent::TOUCH_RELEASE == e.type) {
    setFrame(UNCLICKED);
    _helpActivatedCb();
  }
}

