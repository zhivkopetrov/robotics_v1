//Corresponding header
#include "robo_miner_gui/layout/helpers/RoboMinerLayoutInitHelper.h"

//C system headers

//C++ system headers
#include <algorithm>
#include <numeric>

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

  if (SUCCESS != initPanelHandler(cfg.panelHandlerCfg, outInterface,
          commonInterface, layout)) {
    LOGERR("initPanelHandler() failed");
    return FAILURE;
  }

  if (SUCCESS != initCrystalHandler(cfg.crystalRsrcId, commonInterface,
          layout)) {
    LOGERR("initCrystals() failed");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboMinerLayoutInitHelper::initPanelHandler(
    const PanelHandlerConfig &cfg,
    const RoboMinerLayoutOutInterface &outInterface,
    const RoboCommonLayoutInterface &commonInterface, RoboMinerLayout &layout) {
  PanelHandlerOutInterface panelHandlerOutInterface;
  panelHandlerOutInterface.startGameWonAnimCb =
      commonInterface.startGameWonAnimCb;
  panelHandlerOutInterface.startGameLostAnimCb =
      commonInterface.startGameLostAnimCb;
  panelHandlerOutInterface.startAchievementWonAnimCb =
      commonInterface.startAchievementWonAnimCb;
  panelHandlerOutInterface.fieldMapRevelealedCb =
      outInterface.fieldMapRevelealedCb;

  if (SUCCESS != layout._panelHandler.init(cfg, panelHandlerOutInterface)) {
    LOGERR("Error in _panel.init()");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboMinerLayoutInitHelper::initCrystalHandler(
    uint64_t crystalRsrcId, const RoboCommonLayoutInterface &commonInterface,
    RoboMinerLayout &layout) {
  CrystalHandlerConfig cfg;
  cfg.crystalRsrcId = crystalRsrcId;
  cfg.getFieldDescriptionCb = commonInterface.getFieldDescriptionCb;

  if (SUCCESS != layout._crystalHandler.init(cfg)) {
    LOGERR("Error in _crystalHandler.init()");
    return FAILURE;
  }

  return SUCCESS;
}

