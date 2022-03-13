//Corresponding header
#include "robo_collector_common/layout/controller/buttons/SettingsButton.h"

//System headers

//Other libraries headers
#include "sdl_utils/input/InputEvent.h"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

ErrorCode SettingsButton::init(const SettingsButtonConfig& cfg,
                               const SettingActivatedCb& settingActivatedCb) {
  if (nullptr == settingActivatedCb) {
    LOGERR("Error, nullptr provided for SettingActivatedCb");
    return ErrorCode::FAILURE;
  }
  _settingActivatedCb = settingActivatedCb;

  create(cfg.rsrcId);
  setPosition(cfg.pos);

  return ErrorCode::SUCCESS;
}

void SettingsButton::handleEvent(const InputEvent& e) {
  if (TouchEvent::TOUCH_PRESS == e.type) {
    setFrame(CLICKED);
  } else if (TouchEvent::TOUCH_RELEASE == e.type) {
    setFrame(UNCLICKED);
    _settingActivatedCb();
  }
}

