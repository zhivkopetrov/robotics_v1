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

  if (SUCCESS != initPanelHandler(cfg.panelHandlerCfg, commonInterface,
          layout)) {
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
    const PanelHandlerConfig &cfg, RoboCommonLayoutInterface &commonInterface,
    RoboMinerLayout &layout) {
  PanelHandlerOutInterface outInterface;
  outInterface.startGameWonAnimCb = commonInterface.startGameWonAnimCb;
  outInterface.startGameLostAnimCb = commonInterface.startGameLostAnimCb;
  outInterface.startAchievementWonAnimCb =
      commonInterface.startAchievementWonAnimCb;

  if (SUCCESS != layout._panelHandler.init(cfg, outInterface)) {
    LOGERR("Error in _panel.init()");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboMinerLayoutInitHelper::initCrystalHandler(
    uint64_t crystalRsrcId, RoboCommonLayoutInterface &commonInterface,
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

