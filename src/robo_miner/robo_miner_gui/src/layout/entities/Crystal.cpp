//Corresponding header
#include "robo_miner_gui/layout/entities/Crystal.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "robo_common/layout/field/FieldUtils.h"
#include "sdl_utils/input/InputEvent.h"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t Crystal::init(const CrystalConfig& cfg) {
  if (nullptr == cfg.crystalClickCb) {
    LOGERR("Error, nullptr provided for CrystalClickedCb");
    return FAILURE;
  }
  _crystalClickCb = cfg.crystalClickCb;

  if (nullptr == cfg.getFieldDescriptionCb) {
    LOGERR("Error, nullptr provided for GetFieldDescriptionCb");
    return FAILURE;
  }
  _getFieldDescriptionCb = cfg.getFieldDescriptionCb;

  _fieldPos = cfg.fieldPos;

  create(cfg.rsrcId);
  setFrame(getEnumValue(cfg.type));
  activateAlphaModulation();

  auto pos = FieldUtils::getAbsPos(cfg.fieldPos, _getFieldDescriptionCb());
  pos += cfg.tileOffset;
  setPosition(pos);

  return SUCCESS;
}

void Crystal::handleEvent(const InputEvent& e) {
  if (TouchEvent::TOUCH_RELEASE == e.type) {
    _crystalClickCb(_fieldPos);
  }
}

void Crystal::onLeave([[maybe_unused]]const InputEvent& e) {

}

void Crystal::onReturn([[maybe_unused]]const InputEvent& e) {

}
