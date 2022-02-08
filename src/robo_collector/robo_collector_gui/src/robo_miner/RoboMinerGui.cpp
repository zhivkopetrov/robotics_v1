//Corresponding header
#include "robo_collector_gui/robo_miner/RoboMinerGui.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "sdl_utils/input/InputEvent.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/robo_miner/config/RoboMinerGuiConfig.h"

int32_t RoboMinerGui::init(const RoboMinerGuiConfig& cfg) {
  if (SUCCESS != _field.init(cfg.fieldConfig)) {
    LOGERR("Error, _field.init() failed");
    return FAILURE;
  }

  const auto& fieldData = _field.getFieldData();
  int32_t totalCrystals = 0;
  for (const auto& row : fieldData) {
    for (const auto marker : row) {
      if (cfg.fieldConfig.emptyTileMarker != marker) {
        ++totalCrystals;
      }
    }
  }

  constexpr auto coinOffsetFromTileX = 20;
  constexpr auto coinOffsetFromTileY = 17;
  CrystalConfig crystalCfg;
  crystalCfg.rsrcId = cfg.crystalRsrcId;
  crystalCfg.tileOffset = Point(coinOffsetFromTileX, coinOffsetFromTileY);
  _crystals.resize(totalCrystals);

  const auto fieldDataRows = fieldData.size();
  int32_t crystalId = 0;
  for (size_t row = 0; row < fieldDataRows; ++row) {
    crystalCfg.fieldPos.row = row;
    const auto fieldDataCols = fieldData[row].size();
    for (size_t col = 0; col < fieldDataCols; ++col) {
      const auto marker = fieldData[row][col];
      if (marker == cfg.fieldConfig.emptyTileMarker) {
        continue;
      }

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
  for (const auto& crystal : _crystals) {
    crystal.draw();
  }
}

void RoboMinerGui::handleEvent(const InputEvent &e) {
  for (auto& crystal : _crystals) {
    if (crystal.isInputUnlocked() && crystal.containsEvent(e)) {
      crystal.handleEvent(e);
      break;
    }
  }
}


