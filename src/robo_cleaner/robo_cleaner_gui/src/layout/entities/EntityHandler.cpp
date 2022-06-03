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
                              const FieldDescription &fieldDescr) {
  _rubbish.resize(fieldDescr.emptyTilesCount);

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

      if (rubbishId >= fieldDescr.emptyTilesCount) {
        LOGERR("Internal error, about to populate rubbishId: %d when only %d "
            "should be present", rubbishId, fieldDescr.emptyTilesCount);
        return ErrorCode::FAILURE;
      }

      const auto mappingKey = (row * fieldDescr.cols) + col;
      _fieldPosToRubbishIdMapping[mappingKey] = rubbishId;

      if (ErrorCode::SUCCESS != _rubbish[rubbishId].init(rubbishCfg,
              fieldDescr)) {
        LOGERR("Error, rubbish.init() failed");
        return ErrorCode::FAILURE;
      }
      ++rubbishId;
    }
  }

  return ErrorCode::SUCCESS;
}

void EntityHandler::draw() const {
  for (const auto &rubbish : _rubbish) {
    rubbish.draw();
  }
}

