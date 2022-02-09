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
  if (cfg.helpActivatedCb) {
    LOGERR("Error, nullptr provided for HelpActivatedCb");
    return FAILURE;
  }
  _helpActivatedCb = cfg.helpActivatedCb;

  create(cfg.rsrcId);

  constexpr auto buttonX = 1100;
  constexpr auto buttonY = 600;
  setPosition(buttonX, buttonY);

  return SUCCESS;
}

void HelpButton::handleEvent(const InputEvent& e) {
  if (TouchEvent::KEYBOARD_RELEASE == e.type) {
    _helpActivatedCb();
  }
}

