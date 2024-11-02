//Corresponding header
#include "robo_collector_common/layout/controller/buttons/SettingsButton.h"

//System headers

//Other libraries headers
#include "utils/input/InputEvent.h"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/ErrorCode.h"
#include "utils/log/Log.h"

//Own components headers

ErrorCode SettingsButton::init(const SettingsButtonConfig& cfg,
                               const ToggleDebugInfoCb& toggleDebugInfoCb) {
  if (nullptr == toggleDebugInfoCb) {
    LOGERR("Error, nullptr provided for ToggleDebugInfoCb");
    return ErrorCode::FAILURE;
  }
  _toggleDebugInfoCb = toggleDebugInfoCb;

  create(cfg.rsrcId);
  setPosition(cfg.pos);

  return ErrorCode::SUCCESS;
}

void SettingsButton::handleEvent(const InputEvent& e) {
  if (TouchEvent::TOUCH_PRESS == e.type) {
    setFrame(CLICKED);
  } else if (TouchEvent::TOUCH_RELEASE == e.type) {
    setFrame(UNCLICKED);
    _toggleDebugInfoCb();
  }
}

