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

namespace {
constexpr int32_t SMALL_RUBBISH_ID = 2;
constexpr int32_t BIG_RUBBISH_ID = 3;
constexpr int32_t RUBBISH_TO_FRAME_ID_SUBTRACT_VALUE = 2;
}

ErrorCode EntityHandler::init(const EntityHandlerConfig &cfg,
                              const FieldDescription &fieldDescr) {
  constexpr auto tileOffset = 30;
  RubbishConfig rubbishCfg;
  rubbishCfg.rsrcId = cfg.rubbishRsrcId;
  rubbishCfg.tileOffset = Point(tileOffset, tileOffset);

  for (int32_t row = 0; row < fieldDescr.rows; ++row) {
    rubbishCfg.fieldPos.row = row;
    for (int32_t col = 0; col < fieldDescr.cols; ++col) {
      rubbishCfg.fieldPos.col = col;

      const auto marker = fieldDescr.data[row][col];
      if (!isRubbishMarker(marker)) {
        continue;
      }

      const int32_t rubbishCounter = getRubbishCounter(marker);
      if ((SMALL_RUBBISH_ID == rubbishCounter) ||
          (BIG_RUBBISH_ID == rubbishCounter)) {
        rubbishCfg.frameId = rubbishCounter
            - RUBBISH_TO_FRAME_ID_SUBTRACT_VALUE;

        if (ErrorCode::SUCCESS != createRubbishTile(rubbishCfg, fieldDescr)) {
          LOGERR("createRubbishTile() failed");
          return ErrorCode::FAILURE;
        }
      }

      createCounterText(rubbishCfg.fieldPos, cfg.rubbishFontId, rubbishCounter,
          fieldDescr);
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

ErrorCode EntityHandler::createRubbishTile(const RubbishConfig &rubbishCfg,
                                           const FieldDescription &fieldDescr) {
  auto &elem = _rubbish.emplace_back(Rubbish());
  if (ErrorCode::SUCCESS != elem.init(rubbishCfg, fieldDescr)) {
    LOGERR("Error, rubbish.init() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

void EntityHandler::createCounterText(const FieldPos &fieldPos, uint64_t fontId,
                                      int32_t counterValue,
                                      const FieldDescription &fieldDescr) {
  constexpr auto tileOffsetX = 130;
  constexpr auto tileOffsetY = 5;
  Point offset = Point(tileOffsetX, tileOffsetY);

  auto pos = FieldUtils::getAbsPos(fieldPos, fieldDescr);
  pos += offset;
  auto &text = _tileCounters.emplace_back(Text());
  text.create(fontId, std::to_string(counterValue).c_str(), Colors::RED, pos);
}

