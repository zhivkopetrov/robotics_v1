//Corresponding header
#include "robo_cleaner_gui/layout/entities/EntityHandler.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "robo_common/layout/field/FieldUtils.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_cleaner_gui/defines/RoboCleanerGuiDefines.h"
#include "robo_cleaner_gui/layout/entities/config/EntityHandlerConfig.h"

int32_t EntityHandler::init(
    const EntityHandlerConfig &cfg,
    const GetFieldDescriptionCb &getFieldDescriptionCb) {
  if (nullptr == getFieldDescriptionCb) {
    LOGERR("Error, nullptr provided for GetFieldDescriptionCb");
    return FAILURE;
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
      if (RoboCleanerDefines::OBSTACLE_MARKER == marker) {
        createObstacle(FieldPos(row, col), cfg.obstacleRsrcId);
        continue;
      }

      rubbishCfg.fieldPos.col = col;
      const bool isRubbishTile = isRubbishMarker(marker);
      if (isRubbishTile) {
        if (RoboCleanerDefines::SMALL_RUBISH_MARKER == marker) {
          rubbishCfg.frameId = 0;
        } else if (RoboCleanerDefines::BIG_RUBBISH_MARKER == marker) {
          rubbishCfg.frameId = 1;
        }

        auto &elem = _rubbish.emplace_back(Rubbish());
        if (SUCCESS != elem.init(rubbishCfg, getFieldDescriptionCb)) {
          LOGERR("Error, rubbish.init() failed");
          return FAILURE;
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

  return SUCCESS;
}

void EntityHandler::draw() const {
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

void EntityHandler::createObstacle(const FieldPos &fieldPos, uint64_t rsrcId) {
  constexpr auto tileOffset = 20;
  Point offset = Point(tileOffset, tileOffset);

  auto pos = FieldUtils::getAbsPos(fieldPos, _getFieldDescriptionCb());
  pos += offset;

  auto &obstacle = _obstacles.emplace_back(Image());
  obstacle.create(rsrcId);
  obstacle.setPosition(pos);
}

//int32_t CrystalHandler::init(const CrystalHandlerConfig &cfg) {
//  if (nullptr == cfg.getFieldDescriptionCb) {
//    LOGERR("Error, nullptr provided for GetFieldDescriptionCb");
//    return FAILURE;
//  }
//  _getFieldDescriptionCb = cfg.getFieldDescriptionCb;
//
//  if (SUCCESS != initCrystals(cfg)) {
//    LOGERR("Error, initCrystals() failed");
//    return FAILURE;
//  }
//
//  return SUCCESS;
//}
//
//void CrystalHandler::draw() const {
//  for (const auto &crystal : _crystals) {
//    crystal.draw();
//  }
//}
//
//void CrystalHandler::handleEvent(const InputEvent &e) {
//  for (auto &crystal : _crystals) {
//    if (crystal.isInputUnlocked() && crystal.containsEvent(e)) {
//      crystal.handleEvent(e);
//      break;
//    }
//  }
//}
//
//void CrystalHandler::onCrystalClicked(const FieldPos &fieldPos) {
//  //restore previous opacity
//  for (auto &crystal : _crystals) {
//    crystal.setOpacity(FULL_OPACITY);
//  }
//
//  const auto &fieldDescr = _getFieldDescriptionCb();
//  const std::vector<char> nonCrystalMarkers = { fieldDescr.emptyDataMarker,
//      fieldDescr.hardObstacleMarker };
//  const auto localCrystalSequence = FloodFill::findLocalCrystalSequence(
//      fieldDescr.data, nonCrystalMarkers, fieldPos);
//  for (const auto &pos : localCrystalSequence) {
//    const auto key = (pos.row * fieldDescr.cols) + pos.col;
//    const auto it = _fieldPosToCrystalIdMapping.find(key);
//    if (_fieldPosToCrystalIdMapping.end() == it) {
//      LOGERR("No entry found for fieldPosToCrystalIdMapping key: %d", key);
//      continue;
//    }
//
//    const auto crystalId = it->second;
//    _crystals[crystalId].setOpacity(FULL_OPACITY / 2);
//  }
//}
//
//int32_t CrystalHandler::initCrystals(const CrystalHandlerConfig &cfg) {
//  const auto &fieldDescr = cfg.getFieldDescriptionCb();
//  const auto crystalsCount = std::accumulate(fieldDescr.data.begin(),
//      fieldDescr.data.end(), 0,
//      [](auto count, const auto &row) {
//        return count
//            + std::count_if(std::begin(row), std::end(row), [](auto marker) {
//              return isCrystalMarker(marker);
//            });
//      });
//
//  _crystals.resize(crystalsCount);
//
//  constexpr auto coinOffsetFromTileX = 20;
//  constexpr auto coinOffsetFromTileY = 17;
//  CrystalConfig crystalCfg;
//  crystalCfg.rsrcId = cfg.crystalRsrcId;
//  crystalCfg.tileOffset = Point(coinOffsetFromTileX, coinOffsetFromTileY);
//  crystalCfg.getFieldDescriptionCb = cfg.getFieldDescriptionCb;
//  crystalCfg.crystalClickCb = std::bind(&CrystalHandler::onCrystalClicked, this,
//      std::placeholders::_1);
//
//  int32_t crystalId = 0;
//  for (int32_t row = 0; row < fieldDescr.rows; ++row) {
//    crystalCfg.fieldPos.row = row;
//    for (int32_t col = 0; col < fieldDescr.cols; ++col) {
//      const auto marker = fieldDescr.data[row][col];
//      if (!isCrystalMarker(marker)) {
//        continue;
//      }
//
//      const auto mappingKey = (row * fieldDescr.cols) + col;
//      _fieldPosToCrystalIdMapping[mappingKey] = crystalId;
//
//      crystalCfg.type = getCrystalType(marker);
//      crystalCfg.fieldPos.col = col;
//      if (SUCCESS != _crystals[crystalId].init(crystalCfg)) {
//        LOGERR("Error, _crystal[%d].init() failed", crystalId);
//        return FAILURE;
//      }
//      ++crystalId;
//    }
//  }
//
//  return SUCCESS;
//}

