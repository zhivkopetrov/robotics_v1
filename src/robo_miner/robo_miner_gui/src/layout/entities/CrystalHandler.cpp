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

  createFbo();

  return ErrorCode::SUCCESS;
}

void CrystalHandler::draw() const {
  _allCrystalsFbo.draw();
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

  updateFbo();
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

  constexpr auto crystalToTileRatio = 0.62;
  constexpr auto offBegin = (1.0 - crystalToTileRatio) / 2.0;
  const auto offsetFromTileX = static_cast<int32_t>(offBegin
      * fieldDescr.tileWidth);
  const auto offsetFromTileY = static_cast<int32_t>(offBegin
      * fieldDescr.tileHeight);
  CrystalConfig crystalCfg;
  crystalCfg.rsrcId = cfg.crystalRsrcId;
  crystalCfg.width = crystalToTileRatio * fieldDescr.tileWidth;
  crystalCfg.height = crystalToTileRatio * fieldDescr.tileHeight;
  crystalCfg.tileOffset = Point(offsetFromTileX, offsetFromTileY);
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

void CrystalHandler::createFbo() {
  const auto &fieldDescr = _getFieldDescriptionCb();
  const auto fieldWidth = fieldDescr.cols * fieldDescr.tileWidth;
  const auto fieldHeight = fieldDescr.rows * fieldDescr.tileHeight;
  const auto fieldDimensions = Rectangle(RoboCommonDefines::FIRST_TILE_X_POS,
      RoboCommonDefines::FIRST_TILE_Y_POS, fieldWidth, fieldHeight);

  _allCrystalsFbo.create(fieldDimensions);
  _allCrystalsFbo.activateAlphaModulation();
  _allCrystalsFbo.setResetColor(Colors::FULL_TRANSPARENT);
  updateFbo();
}

void CrystalHandler::updateFbo() {
  _allCrystalsFbo.unlock();
  _allCrystalsFbo.reset();

  for (const auto &crystal : _crystals) {
    _allCrystalsFbo.addWidget(crystal.getButtonTexture());
  }

  _allCrystalsFbo.update();
  _allCrystalsFbo.lock();
}

