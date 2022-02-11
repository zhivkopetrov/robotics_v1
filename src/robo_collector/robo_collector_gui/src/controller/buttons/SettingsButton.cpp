//Corresponding header
#include "robo_collector_gui/controller/buttons/SettingsButton.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "sdl_utils/input/InputEvent.h"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t SettingsButton::init(const SettingsButtonConfig& cfg) {
  if (nullptr == cfg.settingActivatedCb) {
    LOGERR("Error, nullptr provided for SettingActivatedCb");
    return FAILURE;
  }
  _settingActivatedCb = cfg.settingActivatedCb;

  create(cfg.rsrcId);

  constexpr auto buttonX = 1680;
  constexpr auto buttonY = 580;
  setPosition(buttonX, buttonY);

  return SUCCESS;
}

void SettingsButton::handleEvent(const InputEvent& e) {
  if (TouchEvent::TOUCH_PRESS == e.type) {
    setFrame(CLICKED);
  } else if (TouchEvent::TOUCH_RELEASE == e.type) {
    setFrame(UNCLICKED);
    changeGameType();
    _settingActivatedCb(_currGameType);
  }
}

void SettingsButton::changeGameType() {
  switch (_currGameType) {
  case GameType::COLLECTOR:
    _currGameType = GameType::MINER;
    break;
  case GameType::MINER:
    _currGameType = GameType::CLEANER;
    break;
  case GameType::CLEANER:
    _currGameType = GameType::COLLECTOR;
    break;
  default:
    LOGERR("Error, received unsupported GameType: %d",
        getEnumValue(_currGameType));
    break;
  }
}

