//Corresponding header
#include "robo_collector_gui/robo_miner/entities/Crystal.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "sdl_utils/input/InputEvent.h"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/field/FieldUtils.h"

int32_t Crystal::init(const CrystalConfig& cfg) {
  if (nullptr == cfg.onCrystalClickCb) {
    LOGERR("Error, nullptr provided for onCrystalClickCb");
    return FAILURE;
  }
  _onCrystalClickCb = cfg.onCrystalClickCb;
  _fieldPos = cfg.fieldPos;

  create(cfg.rsrcId);
  setFrame(getEnumValue(cfg.type));
  activateAlphaModulation();

  auto pos = FieldUtils::getAbsPos(cfg.fieldPos);
  pos += cfg.tileOffset;
  setPosition(pos);

  return SUCCESS;
}

void Crystal::handleEvent(const InputEvent& e) {
  if (TouchEvent::TOUCH_RELEASE == e.type) {
    _onCrystalClickCb(_fieldPos);
  }
}

void Crystal::onLeave([[maybe_unused]]const InputEvent& e) {

}

void Crystal::onReturn([[maybe_unused]]const InputEvent& e) {

}
