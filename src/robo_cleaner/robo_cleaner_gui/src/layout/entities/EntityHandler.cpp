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

ErrorCode EntityHandler::init(
    const EntityHandlerConfig &cfg,
    const GetFieldDescriptionCb &getFieldDescriptionCb) {
  if (nullptr == getFieldDescriptionCb) {
    LOGERR("Error, nullptr provided for GetFieldDescriptionCb");
    return ErrorCode::FAILURE;
  }
  _getFieldDescriptionCb = getFieldDescriptionCb;

  const auto& fieldDescr = getFieldDescriptionCb();
  constexpr auto tileOffset = 30;
  RubbishConfig rubbishCfg;
  rubbishCfg.rsrcId = cfg.rubbishRsrcId;
  rubbishCfg.tileOffset = Point(tileOffset, tileOffset);
  for (int32_t row = 0; row < fieldDescr.rows; ++row) {
    rubbishCfg.fieldPos.row = row;
    for (int32_t col = 0; col < fieldDescr.cols; ++col) {
      const auto marker = fieldDescr.data[row][col];
      rubbishCfg.fieldPos.col = col;
      const bool isRubbishTile = isRubbishMarker(marker);
      if (isRubbishTile) {
        if (RoboCleanerDefines::SMALL_RUBISH_MARKER == marker) {
          rubbishCfg.frameId = 0;
        } else if (RoboCleanerDefines::BIG_RUBBISH_MARKER == marker) {
          rubbishCfg.frameId = 1;
        }

        auto &elem = _rubbish.emplace_back(Rubbish());
        if (ErrorCode::SUCCESS !=
            elem.init(rubbishCfg, getFieldDescriptionCb)) {
          LOGERR("Error, rubbish.init() failed");
          return ErrorCode::FAILURE;
        }
      }

      //empty tiles should still be cleaned
      if (isRubbishTile || RoboCommonDefines::EMPTY_TILE_MARKER == marker) {
        const auto rubbishCounter = getRubbishCounter(marker);
        if (rubbishCounter) {
          createCounterText(FieldPos(row, col), cfg.rubbishFontId,
              rubbishCounter);
        }
      }
    }
  }

  return ErrorCode::SUCCESS;
}

void EntityHandler::draw() const {
  for (const auto &rubbish : _rubbish) {
    rubbish.draw();
  }
  for (const auto &counter : _tileCounters) {
    counter.draw();
  }
}

void EntityHandler::createCounterText(const FieldPos &fieldPos, uint64_t fontId,
                                      int32_t counterValue) {
  constexpr auto tileOffsetX = 130;
  constexpr auto tileOffsetY = 5;
  Point offset = Point(tileOffsetX, tileOffsetY);

  auto pos = FieldUtils::getAbsPos(fieldPos, _getFieldDescriptionCb());
  pos += offset;
  auto &text = _tileCounters.emplace_back(Text());
  text.create(fontId, std::to_string(counterValue).c_str(), Colors::RED, pos);
}


