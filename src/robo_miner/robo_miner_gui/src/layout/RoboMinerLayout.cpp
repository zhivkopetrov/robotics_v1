//Corresponding header
#include "robo_miner_gui/layout/RoboMinerLayout.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_miner_gui/layout/helpers/RoboMinerLayoutInterfaces.h"
#include "robo_miner_gui/layout/helpers/RoboMinerLayoutInitHelper.h"
#include "robo_miner_gui/helpers/algorithms/FloodFill.h"

using namespace std::placeholders;

int32_t RoboMinerLayout::init(const RoboMinerLayoutConfig &cfg,
                              const RoboMinerLayoutOutInterface &outInterface,
                              RoboMinerLayoutInterface &interface) {
  if (SUCCESS != RoboMinerLayoutInitHelper::init(cfg, outInterface,
          interface.commonLayoutInterface, *this)) {
    LOGERR("Error, RoboMinerLayoutInitHelper::init() failed");
    return FAILURE;
  }

  return SUCCESS;
}

void RoboMinerLayout::deinit() {
  _commonLayout.deinit();
}

void RoboMinerLayout::draw() const {
  _commonLayout.draw();
  _panelHandler.draw();
  for (const auto &crystal : _crystals) {
    crystal.draw();
  }
}

void RoboMinerLayout::handleEvent(const InputEvent &e) {
  for (auto &crystal : _crystals) {
    if (crystal.isInputUnlocked() && crystal.containsEvent(e)) {
      crystal.handleEvent(e);
      break;
    }
  }
}

void RoboMinerLayout::onCrystalClicked(const FieldPos &fieldPos) {
  //restore previous opacity
  for (auto &crystal : _crystals) {
    crystal.setOpacity(FULL_OPACITY);
  }

  const auto &fieldData = _field.getFieldData();
  const int32_t maxCols = static_cast<int32_t>(fieldData[0].size());
  const auto emptyMarker = _field.getEmptyMarker();
  const auto localCrystalSequence = FloodFill::findLocalCrystalSequence(
      fieldData, fieldPos, emptyMarker);
  for (const auto &pos : localCrystalSequence) {
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

