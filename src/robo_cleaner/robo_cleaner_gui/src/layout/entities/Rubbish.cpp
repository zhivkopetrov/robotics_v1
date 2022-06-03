//Corresponding header
#include "robo_cleaner_gui/layout/entities/Rubbish.h"

//System headers

//Other libraries headers
#include "robo_common/layout/field/FieldUtils.h"
#include "manager_utils/drawing/Fbo.h"
#include "utils/Log.h"

//Own components headers
#include "robo_cleaner_gui/defines/RoboCleanerGuiDefines.h"

namespace {
constexpr int32_t SMALL_RUBBISH_ID = 2;
constexpr int32_t BIG_RUBBISH_ID = 3;
constexpr int32_t RUBBISH_TO_FRAME_ID_SUBTRACT_VALUE = 2;
}

ErrorCode Rubbish::init(const RubbishConfig &cfg,
                        const RubbishOutInterface &interface,
                        const FieldDescription &fieldDescr) {
  if (ErrorCode::SUCCESS != createImage(cfg, interface, fieldDescr)) {
    LOGERR("Error, createImage() failed");
    return ErrorCode::FAILURE;
  }

  createText(cfg, fieldDescr);
  return ErrorCode::SUCCESS;
}

void Rubbish::drawOnFbo(Fbo& fbo) const {
#if DEBUG_VISUAL_OVERLAY
  _objApproachOverlay.drawOnFbo(fbo);
#endif //DEBUG_VISUAL_OVERLAY

  fbo.addWidget(_img);
  fbo.addWidget(_counterText);
}

void Rubbish::modifyRubbishWidget(char fieldMarker) {
  const int32_t counterValue = getRubbishCounter(fieldMarker);
  setImageFrame(counterValue);
}

ErrorCode Rubbish::createImage(const RubbishConfig &cfg,
                               const RubbishOutInterface &interface,
                               const FieldDescription &fieldDescr) {
  _img.create(cfg.rsrcId);
  const Point absPos = FieldUtils::getAbsPos(cfg.fieldPos, fieldDescr)
      + cfg.tileOffset;
  _img.setPosition(absPos);

  _img.activateScaling();
  _img.setScaledWidth(cfg.width);
  _img.setScaledHeight(cfg.height);

  setImageFrame(cfg.textCounterValue);

  if (ErrorCode::SUCCESS != createObjOverlay(cfg, interface, fieldDescr)) {
    LOGERR("Error, createObjOverlay() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode Rubbish::createObjOverlay(const RubbishConfig &cfg,
                                    const RubbishOutInterface &interface,
                                    const FieldDescription &fieldDescr) {
  const Point absPos = FieldUtils::getAbsPos(cfg.fieldPos, fieldDescr);

  const ObjectApproachOverlayConfig objOverlayCfg = {
    .preScaledOverlayBoundary = _img.getScaledRect(),
    .upperBoundary =
        Rectangle(absPos, fieldDescr.tileWidth, fieldDescr.tileHeight),
    .scaleFactor = cfg.objApproachOverlayScaleFactor,
    .fieldPos = cfg.fieldPos
  };

  const ObjectApproachOverlayOutInterface objOverLayOutInterface = {
    .objectApproachOverlayTriggeredCb =
        interface.objectApproachOverlayTriggeredCb,
    .containerRedrawCb = interface.containerRedrawCb,
    .collisionWatcher = interface.collisionWatcher
  };

  if (ErrorCode::SUCCESS != _objApproachOverlay.init(objOverlayCfg,
      objOverLayOutInterface)) {
    LOGERR("Error, _objApproachOverlay.init() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
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

void Rubbish::setImageFrame(int32_t counterValue) {
  if ((SMALL_RUBBISH_ID == counterValue) || (BIG_RUBBISH_ID == counterValue)) {
    const int32_t frameIdx = counterValue - RUBBISH_TO_FRAME_ID_SUBTRACT_VALUE;
    _img.setFrame(frameIdx);
  } else {
    _img.hide();
  }
}
