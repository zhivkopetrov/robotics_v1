//Corresponding header
#include "robo_cleaner_gui/layout/entities/Rubbish.h"

//System headers

//Other libraries headers
#include "robo_common/layout/field/FieldUtils.h"
#include "utils/Log.h"

//Own components headers

namespace {
constexpr int32_t SMALL_RUBBISH_ID = 2;
constexpr int32_t BIG_RUBBISH_ID = 3;
constexpr int32_t RUBBISH_TO_FRAME_ID_SUBTRACT_VALUE = 2;
}

ErrorCode Rubbish::init(const RubbishConfig &cfg,
                        const FieldDescription &fieldDescr) {
  createImage(cfg, fieldDescr);
  createText(cfg, fieldDescr);

  return ErrorCode::SUCCESS;
}

void Rubbish::createImage(const RubbishConfig &cfg,
                          const FieldDescription &fieldDescr) {
  _img.create(cfg.rsrcId);
  const Point absPos = FieldUtils::getAbsPos(cfg.fieldPos, fieldDescr)
      + cfg.tileOffset;
  _img.setPosition(absPos);

  _img.activateScaling();
  _img.setScaledWidth(cfg.width);
  _img.setScaledHeight(cfg.height);

  if ( (SMALL_RUBBISH_ID == cfg.textCounterValue) || (BIG_RUBBISH_ID
      == cfg.textCounterValue)) {
    const int32_t frameIdx = cfg.textCounterValue
        - RUBBISH_TO_FRAME_ID_SUBTRACT_VALUE;
    _img.setFrame(frameIdx);
  } else {
    _img.hide();
  }
}

void Rubbish::createText(const RubbishConfig &cfg,
                         const FieldDescription &fieldDescr) {
  constexpr double textToTileRatio = 0.2;
  const int32_t offsetFromTileX = static_cast<int32_t>(0.75
      * fieldDescr.tileWidth);
  const int32_t offsetFromTileY = static_cast<int32_t>(0.1
      * fieldDescr.tileHeight);

  const int32_t maxScaledWidth = textToTileRatio * fieldDescr.tileWidth;
  const int32_t maxScaledHeight = textToTileRatio * fieldDescr.tileHeight;
  const Point offset = Point(offsetFromTileX, offsetFromTileY);

  const Point absPos = FieldUtils::getAbsPos(cfg.fieldPos, fieldDescr) + offset;
  _counterText.create(cfg.counterTextFontId,
      std::to_string(cfg.textCounterValue).c_str(), Colors::RED, absPos);
  _counterText.activateScaling();
  _counterText.setMaxScalingWidth(maxScaledWidth);
  _counterText.setMaxScalingHeight(maxScaledHeight);
}

void Rubbish::draw() const {
  _img.draw();
  _counterText.draw();
}
