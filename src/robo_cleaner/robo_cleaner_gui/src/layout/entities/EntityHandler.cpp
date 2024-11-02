//Corresponding header
#include "robo_cleaner_gui/layout/entities/EntityHandler.h"

//System headers

//Other libraries headers
#include "robo_cleaner_common/defines/RoboCleanerDefines.h"
#include "robo_common/layout/field/FieldUtils.h"
#include "utils/drawing/WidgetAligner.h"
#include "utils/ErrorCode.h"
#include "utils/log/Log.h"

//Own components headers
#include "robo_cleaner_gui/layout/entities/config/EntityHandlerConfig.h"

ErrorCode EntityHandler::init(const EntityHandlerConfig &cfg,
                              const EntityHandlerOutInterface &interface,
                              const FieldDescription &fieldDescr) {
  _fieldCols = fieldDescr.cols;
  createEntityHandlerFbo(fieldDescr);

  if (ErrorCode::SUCCESS != createRubbishEntities(cfg, interface, fieldDescr)) {
    LOGERR("Error, createRubbishEntities() failed");
    return ErrorCode::FAILURE;
  }

  createChargingStation(cfg.chargingStationRsrcId, cfg.playerStartPosition,
      fieldDescr);

  updateEntityHandlerFbo();
  return ErrorCode::SUCCESS;
}

void EntityHandler::draw() const {
  _fbo.draw();
}

void EntityHandler::modifyRubbishWidget(const FieldPos &fieldPos,
                                        char fieldMarker) {
  const int32_t key = (fieldPos.row * _fieldCols) + fieldPos.col;
  const auto it = _fieldPosToRubbishIdMapping.find(key);
  if (_fieldPosToRubbishIdMapping.end() == it) {
    LOGERR("No entry found for _fieldPosToRubbishIdMapping key: %d", key);
    return;
  }

  const int32_t rubbishId = it->second;
  _rubbish[rubbishId].modifyRubbishWidget(fieldMarker);
  updateEntityHandlerFbo();
}

void EntityHandler::createEntityHandlerFbo(const FieldDescription &fieldDescr) {
  const auto fieldWidth = fieldDescr.cols * fieldDescr.tileWidth;
  const auto fieldHeight = fieldDescr.rows * fieldDescr.tileHeight;
  const auto entityHandlerDimensios = Rectangle(
      RoboCommonDefines::FIRST_TILE_X_POS, RoboCommonDefines::FIRST_TILE_Y_POS,
      fieldWidth, fieldHeight);
  _fbo.create(entityHandlerDimensios);
  _fbo.activateAlphaModulation();
  _fbo.setResetColor(Colors::FULL_TRANSPARENT);
}

ErrorCode EntityHandler::createRubbishEntities(
    const EntityHandlerConfig &cfg, const EntityHandlerOutInterface &interface,
    const FieldDescription &fieldDescr) {
  //remove the tile (bottom-right corner) for the starting position of the robot
  const int32_t rubbishEntitiesCount = fieldDescr.emptyTilesCount - 1;
  _rubbish.resize(rubbishEntitiesCount);

  const RubbishOutInterface rubbishOutInterface = {
      .objectApproachOverlayTriggeredCb =
          interface.objectApproachOverlayTriggeredCb, .containerRedrawCb =
          std::bind(&EntityHandler::updateEntityHandlerFbo, this),
      .collisionWatcher = interface.collisionWatcher };

  constexpr double rubbishToTileRatio = 0.5;
  constexpr double offBegin = (1.0 - rubbishToTileRatio) / 2.0;
  const int32_t offsetFromTileX = static_cast<int32_t>(offBegin
      * fieldDescr.tileWidth);
  const int32_t offsetFromTileY = static_cast<int32_t>(offBegin
      * fieldDescr.tileHeight);

  RubbishConfig rubbishCfg;
  rubbishCfg.rsrcId = cfg.rubbishRsrcId;
  rubbishCfg.counterTextFontId = cfg.rubbishFontId;
  rubbishCfg.tileOffset = Point(offsetFromTileX, offsetFromTileY);
  rubbishCfg.width = rubbishToTileRatio * fieldDescr.tileWidth;
  rubbishCfg.height = rubbishToTileRatio * fieldDescr.tileHeight;
  rubbishCfg.objApproachOverlayScaleFactor = 1.2;

  int32_t rubbishId = 0;
  for (int32_t row = 0; row < fieldDescr.rows; ++row) {
    rubbishCfg.fieldPos.row = row;
    for (int32_t col = 0; col < fieldDescr.cols; ++col) {
      rubbishCfg.fieldPos.col = col;

      const auto marker = fieldDescr.data[row][col];
      if (!isRubbishMarker(marker)) {
        continue;
      }
      rubbishCfg.textCounterValue = getRubbishCounter(marker);

      if (rubbishEntitiesCount == rubbishId) {
        break; //skip the last index
      }

      const auto mappingKey = (row * fieldDescr.cols) + col;
      _fieldPosToRubbishIdMapping[mappingKey] = rubbishId;

      if (ErrorCode::SUCCESS != _rubbish[rubbishId].init(rubbishCfg,
              rubbishOutInterface, fieldDescr)) {
        LOGERR("Error, rubbish.init() failed");
        return ErrorCode::FAILURE;
      }
      ++rubbishId;
    }
  }

  return ErrorCode::SUCCESS;
}

void EntityHandler::createChargingStation(uint64_t rsrcId,
                                          const FieldPos &fieldPos,
                                          const FieldDescription &fieldDescr) {
  _chargingStationImg.create(rsrcId);

  constexpr double imgToTileRatio = 0.7;
  const int32_t originalWidth = _chargingStationImg.getImageWidth();
  const int32_t originalHeight = _chargingStationImg.getImageHeight();
  const double widthToHeightRatio =
      static_cast<double>(originalWidth) / originalHeight;
  const int32_t scaledWidth = static_cast<int32_t>(fieldDescr.tileWidth
      * widthToHeightRatio * imgToTileRatio);
  const int32_t scaledHeight = static_cast<int32_t>(fieldDescr.tileHeight
      * imgToTileRatio);

  const Point absTilePos = FieldUtils::getAbsPos(fieldPos, fieldDescr);
  const Rectangle tileBoundary =
      Rectangle(absTilePos, fieldDescr.tileWidth, fieldDescr.tileHeight);
  const Point imgCenteredPos = WidgetAligner::getPosition(scaledWidth,
      scaledHeight, tileBoundary, WidgetAlignment::CENTER_CENTER);

  _chargingStationImg.activateScaling();
  _chargingStationImg.setScaledWidth(scaledWidth);
  _chargingStationImg.setScaledHeight(scaledHeight);
  _chargingStationImg.setPosition(imgCenteredPos);
}

void EntityHandler::updateEntityHandlerFbo() {
  _fbo.unlock();
  _fbo.reset();

  _fbo.addWidget(_chargingStationImg);
  for (const auto &rubbish : _rubbish) {
    rubbish.drawOnFbo(_fbo);
  }

  _fbo.update();
  _fbo.lock();
}

