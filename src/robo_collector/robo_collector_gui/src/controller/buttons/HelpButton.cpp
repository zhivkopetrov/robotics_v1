//Corresponding header
#include "robo_collector_gui/controller/buttons/HelpButton.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "sdl_utils/input/InputEvent.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t HelpButton::init(const HelpButtonConfig& cfg) {
  if (nullptr == cfg.helpActivatedCb) {
    LOGERR("Error, nullptr provided for HelpActivatedCb");
    return FAILURE;
  }
  _helpActivatedCb = cfg.helpActivatedCb;

  create(cfg.rsrcId);

  constexpr auto buttonX = 1680;
  constexpr auto buttonY = 660;
  setPosition(buttonX, buttonY);

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

