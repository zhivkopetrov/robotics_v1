//Corresponding header
#include "robo_miner_gui/layout/helpers/RoboMinerLayoutInitHelper.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_miner_gui/layout/helpers/RoboMinerLayoutInterfaces.h"
#include "robo_miner_gui/layout/config/RoboMinerLayoutConfig.h"
#include "robo_miner_gui/layout/RoboMinerLayout.h"

using namespace std::placeholders;

int32_t RoboMinerLayoutInitHelper::init(
    const RoboMinerLayoutConfig &cfg,
    const RoboMinerLayoutOutInterface &outInterface,
    RoboCommonLayoutInterface &commonInterface, //out param
    RoboMinerLayout &layout) {
  RoboCommonLayoutOutInterface commonOutInterface;
  commonOutInterface.collisionWatcher = outInterface.collisionWatcher;
  commonOutInterface.finishRobotActCb = outInterface.finishRobotActCb;
  commonOutInterface.playerDamageCb = std::bind(
      &PanelHandler::decreaseHealthIndicator, &layout._panelHandler, _1);
  commonOutInterface.shutdownGameCb = outInterface.shutdownGameCb;

  if (SUCCESS != layout._commonLayout.init(cfg.commonLayoutCfg,
          commonOutInterface, commonInterface)) {
    LOGERR("_commonLayout.init() failed");
    return FAILURE;
  }

  if (SUCCESS != initPanelHandler(cfg.panelHandlerCfg, commonInterface,
          layout)) {
    LOGERR("initPanelHandler() failed");
    return FAILURE;
  }

  if (SUCCESS != layout._field.init(cfg.fieldCfg)) {
    LOGERR("Error, _field.init() failed");
    return FAILURE;
  }

  if (SUCCESS !=
      initCrystals(cfg.crystalRsrcId, cfg.fieldCfg.emptyTileMarker, layout)) {
    LOGERR("initCrystals() failed");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboMinerLayoutInitHelper::initPanelHandler(
    const PanelHandlerConfig &cfg, RoboCommonLayoutInterface &commonInterface,
    RoboMinerLayout &layout) {
  PanelHandlerOutInterface outInterface;
  outInterface.startGameWonAnimCb = commonInterface.startGameWonAnimCb;
  outInterface.startGameLostAnimCb = commonInterface.startGameLostAnimCb;

  if (SUCCESS != layout._panelHandler.init(cfg, outInterface)) {
    LOGERR("Error in _panel.init()");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboMinerLayoutInitHelper::initCrystals(uint64_t crystalRsrcId,
                                                char emptyTileMarker,
                                                RoboMinerLayout &layout) {
  const auto &fieldData = layout._field.getFieldData();
  int32_t totalCrystals = 0;
  for (const auto &row : fieldData) {
    for (const auto marker : row) {
      if (emptyTileMarker != marker) {
        ++totalCrystals;
      }
    }
  }
  layout._crystals.resize(totalCrystals);

  constexpr auto coinOffsetFromTileX = 20;
  constexpr auto coinOffsetFromTileY = 17;
  CrystalConfig crystalCfg;
  crystalCfg.rsrcId = crystalRsrcId;
  crystalCfg.tileOffset = Point(coinOffsetFromTileX, coinOffsetFromTileY);
  crystalCfg.crystalClickCb =
      std::bind(&RoboMinerLayout::onCrystalClicked, &layout, _1);

  const auto fieldDataRows = fieldData.size();
  int32_t crystalId = 0;
  for (size_t row = 0; row < fieldDataRows; ++row) {
    crystalCfg.fieldPos.row = row;
    const auto fieldDataCols = fieldData[row].size();
    for (size_t col = 0; col < fieldDataCols; ++col) {
      const auto marker = fieldData[row][col];
      if (marker == emptyTileMarker) {
        continue;
      }

      const auto mappingKey = static_cast<int32_t>( (row * fieldDataCols) + col);
      layout._fieldPosToCrystalIdMapping[mappingKey] = crystalId;

      crystalCfg.type = getCrystalType(marker);
      crystalCfg.fieldPos.col = col;
      if (SUCCESS != layout._crystals[crystalId].init(crystalCfg)) {
        LOGERR("Error, _crystal[%d].init() failed", crystalId);
        return FAILURE;
      }
      ++crystalId;
    }
  }

  return SUCCESS;
}

