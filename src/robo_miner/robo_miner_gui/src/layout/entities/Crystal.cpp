//Corresponding header
#include "robo_miner_gui/layout/entities/Crystal.h"

//System headers

//Other libraries headers
#include "robo_common/layout/field/FieldUtils.h"
#include "utils/input/InputEvent.h"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/Log.h"

//Own components headers

ErrorCode Crystal::init(const CrystalConfig& cfg) {
  if (nullptr == cfg.crystalClickCb) {
    LOGERR("Error, nullptr provided for CrystalClickedCb");
    return ErrorCode::FAILURE;
  }
  _crystalClickCb = cfg.crystalClickCb;

  if (nullptr == cfg.getFieldDescriptionCb) {
    LOGERR("Error, nullptr provided for GetFieldDescriptionCb");
    return ErrorCode::FAILURE;
  }
  _getFieldDescriptionCb = cfg.getFieldDescriptionCb;

  _fieldPos = cfg.fieldPos;

  create(cfg.rsrcId);
  setFrame(getEnumValue(cfg.type));
  activateAlphaModulation();

  auto& buttonTexture = getButtonTextureMutable();
  buttonTexture.activateScaling();

  auto pos = FieldUtils::getAbsPos(cfg.fieldPos, _getFieldDescriptionCb());
  pos += cfg.tileOffset;
  setPosition(pos);
  buttonTexture.setScaledWidth(cfg.width);
  buttonTexture.setScaledHeight(cfg.height);

  setEventCaptureRect(buttonTexture.getScaledRect());

  return ErrorCode::SUCCESS;
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
