//Corresponding header
#include "robo_collector_gui/robo_cleaner/entities/Rubbish.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/field/FieldUtils.h"

int32_t Rubbish::init(const RubbishConfig& cfg) {
  _img.create(cfg.rsrcId);
  auto pos = FieldUtils::getAbsPos(cfg.fieldPos);
  pos += cfg.tileOffset;
  _img.setPosition(pos);
  _img.setFrame(cfg.frameId);

  return SUCCESS;
}

void Rubbish::draw() const {
  _img.draw();
}
