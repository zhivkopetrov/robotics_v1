//Corresponding header
#include "robo_cleaner_gui/layout/entities/EntityHandler.h"

//System headers

//Other libraries headers
#include "robo_common/layout/field/FieldUtils.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_cleaner_gui/defines/RoboCleanerGuiDefines.h"
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
          interface.objectApproachOverlayTriggeredCb,
      .containerRedrawCb =
          std::bind(&EntityHandler::updateEntityHandlerFbo, this),
      .collisionWatcher = interface.collisionWatcher
  };

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

void EntityHandler::updateEntityHandlerFbo() {
  _fbo.unlock();
  _fbo.reset();

  for (const auto &rubbish : _rubbish) {
    rubbish.drawOnFbo(_fbo);
  }

  _fbo.update();
  _fbo.lock();
}

