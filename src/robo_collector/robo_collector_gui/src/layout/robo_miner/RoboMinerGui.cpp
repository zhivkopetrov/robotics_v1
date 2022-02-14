//Corresponding header
#include "robo_collector_gui/layout/robo_miner/RoboMinerGui.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "sdl_utils/input/InputEvent.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/layout/robo_miner/config/RoboMinerGuiConfig.h"
#include "robo_collector_gui/layout/robo_miner/algorithms/FloodFill.h"

int32_t RoboMinerGui::init(const RoboMinerGuiConfig &cfg) {
  if (SUCCESS != _field.init(cfg.fieldCfg)) {
    LOGERR("Error, _field.init() failed");
    return FAILURE;
  }

  const auto &fieldData = _field.getFieldData();
  int32_t totalCrystals = 0;
  for (const auto &row : fieldData) {
    for (const auto marker : row) {
      if (cfg.fieldCfg.emptyTileMarker != marker) {
        ++totalCrystals;
      }
    }
  }
  _crystals.resize(totalCrystals);

  constexpr auto coinOffsetFromTileX = 20;
  constexpr auto coinOffsetFromTileY = 17;
  CrystalConfig crystalCfg;
  crystalCfg.rsrcId = cfg.crystalRsrcId;
  crystalCfg.tileOffset = Point(coinOffsetFromTileX, coinOffsetFromTileY);
  crystalCfg.onCrystalClickCb = std::bind(&RoboMinerGui::onCrystalClicked, this,
      std::placeholders::_1);

  const auto fieldDataRows = fieldData.size();
  int32_t crystalId = 0;
  for (size_t row = 0; row < fieldDataRows; ++row) {
    crystalCfg.fieldPos.row = row;
    const auto fieldDataCols = fieldData[row].size();
    for (size_t col = 0; col < fieldDataCols; ++col) {
      const auto marker = fieldData[row][col];
      if (marker == cfg.fieldCfg.emptyTileMarker) {
        continue;
      }

      const auto mappingKey = static_cast<int32_t>((row * fieldDataCols) + col);
      _fieldPosToCrystalIdMapping[mappingKey] = crystalId;

      crystalCfg.type = getCrystalType(marker);
      crystalCfg.fieldPos.col = col;
      if (SUCCESS != _crystals[crystalId].init(crystalCfg)) {
        LOGERR("Error, _crystal[%d].init() failed", crystalId);
        return FAILURE;
      }
      ++crystalId;
    }
  }

  return SUCCESS;
}

void RoboMinerGui::deinit() {

}

void RoboMinerGui::draw() const {
  for (const auto &crystal : _crystals) {
    crystal.draw();
  }
}

void RoboMinerGui::handleEvent(const InputEvent &e) {
  for (auto &crystal : _crystals) {
    if (crystal.isInputUnlocked() && crystal.containsEvent(e)) {
      crystal.handleEvent(e);
      break;
    }
  }
}

void RoboMinerGui::onCrystalClicked(const FieldPos &fieldPos) {
  //restore previous opacity
  for (auto &crystal : _crystals) {
    crystal.setOpacity(FULL_OPACITY);
  }

  const auto& fieldData = _field.getFieldData();
  const int32_t maxCols = static_cast<int32_t>(fieldData[0].size());
  const auto emptyMarker = _field.getEmptyMarker();
  const auto localCrystalSequence =
      FloodFill::findLocalCrystalSequence(fieldData, fieldPos, emptyMarker);
  for (const auto& pos : localCrystalSequence) {
    const auto key = (pos.row * maxCols) + pos.col;
    const auto it = _fieldPosToCrystalIdMapping.find(key);
    if (_fieldPosToCrystalIdMapping.end() == it) {
      LOGERR("No entry found for fieldPosToCrystalIdMapping key: %d", key);
      continue;
    }

    const auto crystalId = it->second;
    _crystals[crystalId].setOpacity(FULL_OPACITY / 2);
  }
}

