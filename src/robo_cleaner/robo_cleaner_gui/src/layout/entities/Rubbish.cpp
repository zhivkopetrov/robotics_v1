//Corresponding header
#include "robo_cleaner_gui/layout/entities/Rubbish.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "robo_common/layout/field/FieldUtils.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t Rubbish::init(const RubbishConfig& cfg,
                      const GetFieldDescriptionCb& getFieldDescriptionCb) {
  if (nullptr == getFieldDescriptionCb) {
    LOGERR("Error, nullptr provided for GetFieldDescriptionCb");
    return FAILURE;
  }

  _img.create(cfg.rsrcId);
  auto pos = FieldUtils::getAbsPos(cfg.fieldPos, getFieldDescriptionCb());
  pos += cfg.tileOffset;
  _img.setPosition(pos);
  _img.setFrame(cfg.frameId);

  return SUCCESS;
}

void Rubbish::draw() const {
  _img.draw();
}
