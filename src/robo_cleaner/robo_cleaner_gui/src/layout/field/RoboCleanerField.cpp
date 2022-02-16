//Corresponding header
#include "robo_cleaner_gui/layout/field/RoboCleanerField.h"

//C system headers

//C++ system headers
#include <string>

//Other libraries headers
#include "robo_common/layout/field/FieldUtils.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_cleaner_gui/layout/field/config/RoboCleanerFieldConfig.h"

namespace {
//TODO remove this temporary hack
int32_t generateMapConfig(FieldData& data) {
  const auto rows = data.size();
  if (rows != RoboCommonDefines::FIELD_ROWS) {
    const auto cols = data[0].size();
    if (cols != RoboCommonDefines::FIELD_COLS) {
      LOGERR("Only %zu%zu is matrix is supported at the moment", rows, cols);
      return FAILURE;
    }
  }

  data = {
    { 'x', 'x', '.', '.', '.', '.', '.'},
    { 'x', 'x', 'r', 'r', '.', 'R', 'x'},
    { '.', '.', 'r', 'r', '.', '.', 'x'},
    { '.', '.', '.', '.', '.', 'R', '.'},
    { '.', 'R', 'R', '.', '.', '.', '.'},
    { 'x', 'x', '.', '.', '.', '.', '.'}
  };
  return SUCCESS;
}

bool isRubbishMarker(char marker) {
  return ('r' == marker) || ('R' == marker);
}

int32_t getRubbishCounter(char marker) {
  switch (marker) {
  case '.':
    return 1;
  case 'r':
    return 2;
  case 'R':
    return 3;
  default:
    LOGERR("Error, received invalid marker: %c", marker);
    return 0;
  }
}
}

int32_t RoboCleanerField::init(const RoboCleanerFieldConfig &cfg) {
  if (0 >= cfg.rows || 0 >= cfg.cols) {
    LOGERR("Invalid configuration, rows: %d, cols: %d. Both 'rows' and 'cols' "
        "needs to be positive number", cfg.rows, cfg.cols);
    return FAILURE;
  }

  _emptyDataMarker = cfg.emptyTileMarker;
  _fieldData.resize(cfg.rows);
  for (int32_t row = 0; row < cfg.rows; ++row) {
    _fieldData[row].resize(cfg.cols, cfg.emptyTileMarker);
  }

  if (SUCCESS != generateMapConfig(_fieldData)) {
    LOGERR("Error, generateMapConfig() failed");
    return FAILURE;
  }

  if (SUCCESS != initEntities(cfg)) {
    LOGERR("Error, generateMapConfig() failed");
    return FAILURE;
  }

  return SUCCESS;
}

const FieldData& RoboCleanerField::getFieldData() const {
  return _fieldData;
}

char RoboCleanerField::getEmptyMarker() const {
  return _emptyDataMarker;
}

void RoboCleanerField::draw() const {
  for (const auto &rubbish : _rubbish) {
    rubbish.draw();
  }
  for (const auto &counter : _tileCounters) {
    counter.draw();
  }
  for (const auto &obstacle : _obstacles) {
    obstacle.draw();
  }
}

int32_t RoboCleanerField::initEntities(const RoboCleanerFieldConfig &cfg) {
  constexpr auto tileOffset = 30;
  RubbishConfig rubbishCfg;
  rubbishCfg.rsrcId = cfg.rubbishRsrcId;
  rubbishCfg.tileOffset = Point(tileOffset, tileOffset);
  const int32_t rows = static_cast<int32_t>(_fieldData.size());
  for (int32_t row = 0; row < rows; ++row) {
    const int32_t cols = static_cast<int32_t>(_fieldData[0].size());
    rubbishCfg.fieldPos.row = row;
    for (int32_t col = 0; col < cols; ++col) {
      const auto marker = _fieldData[row][col];
      if ('x' == marker) {
        createObstacle(FieldPos(row, col), cfg.obstacleRsrcId);
        continue;
      }

      rubbishCfg.fieldPos.col = col;
      if ('r' == marker) {
        rubbishCfg.frameId = 0;
      } else if ('R' == marker) {
        rubbishCfg.frameId = 1;
      }

      if (isRubbishMarker(marker)) {
        auto &elem = _rubbish.emplace_back(Rubbish());
        if (SUCCESS != elem.init(rubbishCfg)) {
          LOGERR("Error, rubbish.init() failed");
          return FAILURE;
        }
      }

      const auto rubbishCounter = getRubbishCounter(marker);
      if (rubbishCounter) {
        createCounterText(FieldPos(row, col), cfg.rubbishFontId,
            rubbishCounter);
      }
    }
  }

  return SUCCESS;
}

void RoboCleanerField::createCounterText(const FieldPos &fieldPos,
                                       uint64_t fontId, int32_t counterValue) {
  constexpr auto tileOffsetX = 130;
  constexpr auto tileOffsetY = 5;
  Point offset = Point(tileOffsetX, tileOffsetY);

  auto pos = FieldUtils::getAbsPos(fieldPos);
  pos += offset;
  auto &text = _tileCounters.emplace_back(Text());
  text.create(fontId, std::to_string(counterValue).c_str(), Colors::RED, pos);
}

void RoboCleanerField::createObstacle(const FieldPos &fieldPos, uint64_t rsrcId) {
  constexpr auto tileOffset = 20;
  Point offset = Point(tileOffset, tileOffset);

  auto pos = FieldUtils::getAbsPos(fieldPos);
  pos += offset;

  auto& obstacle = _obstacles.emplace_back(Image());
  obstacle.create(rsrcId);
  obstacle.setPosition(pos);
}

