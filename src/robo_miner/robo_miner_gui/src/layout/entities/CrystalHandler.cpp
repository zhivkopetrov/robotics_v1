//Corresponding header
#include "robo_miner_gui/layout/entities/CrystalHandler.h"

//System headers
#include <algorithm>
#include <numeric>

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_miner_gui/defines/RoboMinerGuiDefines.h"
#include "robo_miner_gui/helpers/algorithms/FloodFill.h"

ErrorCode CrystalHandler::init(const CrystalHandlerConfig &cfg) {
  if (nullptr == cfg.getFieldDescriptionCb) {
    LOGERR("Error, nullptr provided for GetFieldDescriptionCb");
    return ErrorCode::FAILURE;
  }
  _getFieldDescriptionCb = cfg.getFieldDescriptionCb;

  if (ErrorCode::SUCCESS != initCrystals(cfg)) {
    LOGERR("Error, initCrystals() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

void CrystalHandler::draw() const {
  for (const auto &crystal : _crystals) {
    crystal.draw();
  }
}

void CrystalHandler::handleEvent(const InputEvent &e) {
  for (auto &crystal : _crystals) {
    if (crystal.isInputUnlocked() && crystal.containsEvent(e)) {
      crystal.handleEvent(e);
      break;
    }
  }
}

void CrystalHandler::onCrystalClicked(const FieldPos &fieldPos) {
  //restore previous opacity
  for (auto &crystal : _crystals) {
    crystal.setOpacity(FULL_OPACITY);
  }

  const auto &fieldDescr = _getFieldDescriptionCb();
  std::vector<char> nonCrystalMarkers = fieldDescr.obstacleMarkers;
  nonCrystalMarkers.push_back(fieldDescr.emptyDataMarker);

  const auto localCrystalSequence = FloodFill::findLocalCrystalSequence(
      fieldDescr.data, nonCrystalMarkers, fieldPos);
  for (const auto &pos : localCrystalSequence) {
    const auto key = (pos.row * fieldDescr.cols) + pos.col;
    const auto it = _fieldPosToCrystalIdMapping.find(key);
    if (_fieldPosToCrystalIdMapping.end() == it) {
      LOGERR("No entry found for fieldPosToCrystalIdMapping key: %d", key);
      continue;
    }

    const auto crystalId = it->second;
    _crystals[crystalId].setOpacity(FULL_OPACITY / 2);
  }
}

ErrorCode CrystalHandler::initCrystals(const CrystalHandlerConfig &cfg) {
  const auto &fieldDescr = cfg.getFieldDescriptionCb();
  const auto crystalsCount = std::accumulate(fieldDescr.data.begin(),
      fieldDescr.data.end(), 0,
      [](auto count, const auto &row) {
        return count
            + std::count_if(std::begin(row), std::end(row), [](auto marker) {
              return isCrystalMarker(marker);
            });
      });

  _crystals.resize(crystalsCount);

  constexpr auto coinOffsetFromTileX = 20;
  constexpr auto coinOffsetFromTileY = 17;
  CrystalConfig crystalCfg;
  crystalCfg.rsrcId = cfg.crystalRsrcId;
  crystalCfg.tileOffset = Point(coinOffsetFromTileX, coinOffsetFromTileY);
  crystalCfg.getFieldDescriptionCb = cfg.getFieldDescriptionCb;
  crystalCfg.crystalClickCb = std::bind(&CrystalHandler::onCrystalClicked, this,
      std::placeholders::_1);

  int32_t crystalId = 0;
  for (int32_t row = 0; row < fieldDescr.rows; ++row) {
    crystalCfg.fieldPos.row = row;
    for (int32_t col = 0; col < fieldDescr.cols; ++col) {
      const auto marker = fieldDescr.data[row][col];
      if (!isCrystalMarker(marker)) {
        continue;
      }

      const auto mappingKey = (row * fieldDescr.cols) + col;
      _fieldPosToCrystalIdMapping[mappingKey] = crystalId;

      crystalCfg.type = getCrystalType(marker);
      crystalCfg.fieldPos.col = col;
      if (ErrorCode::SUCCESS != _crystals[crystalId].init(crystalCfg)) {
        LOGERR("Error, _crystal[%d].init() failed", crystalId);
        return ErrorCode::FAILURE;
      }
      ++crystalId;
    }
  }

  return ErrorCode::SUCCESS;
}

