//Corresponding header
#include "robo_cleaner_gui/layout/entities/Rubbish.h"

//System headers

//Other libraries headers
#include "robo_common/layout/field/FieldUtils.h"
#include "utils/Log.h"

//Own components headers

ErrorCode Rubbish::init(const RubbishConfig& cfg,
                        const FieldDescription &FieldDescr) {
  _img.create(cfg.rsrcId);
  const auto pos =
      FieldUtils::getAbsPos(cfg.fieldPos, FieldDescr) + cfg.tileOffset;
  _img.setPosition(pos);
  _img.setFrame(cfg.frameId);

  _img.activateScaling();
  _img.setScaledWidth(cfg.width);
  _img.setScaledHeight(cfg.height);

  return ErrorCode::SUCCESS;
}

void Rubbish::draw() const {
  _img.draw();
}
