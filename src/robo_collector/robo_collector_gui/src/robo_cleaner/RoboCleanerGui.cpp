//Corresponding header
#include "robo_collector_gui/robo_cleaner/RoboCleanerGui.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "robo_common/layout/field/FieldUtils.h"
#include "sdl_utils/input/InputEvent.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/robo_cleaner/config/RoboCleanerGuiConfig.h"

namespace {
//TODO remove this temporary hack
FieldData generateMapConfig() {
  return {
    { 'x', 'x', '.', '.', '.', '.', '.'},
    { 'x', 'x', 'r', 'r', '.', 'R', 'x'},
    { '.', '.', 'r', 'r', '.', '.', 'x'},
    { '.', '.', '.', '.', '.', 'R', '.'},
    { '.', 'R', 'R', '.', '.', '.', '.'},
    { 'x', 'x', '.', '.', '.', '.', '.'},
  };
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
    return 0;
  }
}
}

int32_t RoboCleanerGui::init(const RoboCleanerGuiConfig &cfg) {
  if (SUCCESS != _energyPanel.init(cfg.energyPanelCfg)) {
    LOGERR("Error, _energyPanel.init() failed");
    return FAILURE;
  }

  _fieldData = generateMapConfig();
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

void RoboCleanerGui::deinit() {

}

void RoboCleanerGui::draw() const {
  _energyPanel.draw();
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

void RoboCleanerGui::createCounterText(const FieldPos &fieldPos,
                                       uint64_t fontId, int32_t counterValue) {
  constexpr auto tileOffsetX = 130;
  constexpr auto tileOffsetY = 5;
  Point offset = Point(tileOffsetX, tileOffsetY);

  auto pos = FieldUtils::getAbsPos(fieldPos);
  pos += offset;
  auto &text = _tileCounters.emplace_back(Text());
  text.create(fontId, std::to_string(counterValue).c_str(), Colors::RED, pos);
}

void RoboCleanerGui::createObstacle(const FieldPos &fieldPos, uint64_t rsrcId) {
  constexpr auto tileOffset = 20;
  Point offset = Point(tileOffset, tileOffset);

  auto pos = FieldUtils::getAbsPos(fieldPos);
  pos += offset;

  auto& obstacle = _obstacles.emplace_back(Image());
  obstacle.create(rsrcId);
  obstacle.setPosition(pos);
}

