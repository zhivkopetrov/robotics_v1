//Corresponding header
#include "robo_miner_gui/layout/helpers/RoboMinerLayoutInitHelper.h"

//System headers
#include <algorithm>
#include <numeric>

//Other libraries headers
#include "utils/Log.h"

//Own components headers
#include "robo_miner_gui/layout/helpers/RoboMinerLayoutInterfaces.h"
#include "robo_miner_gui/layout/config/RoboMinerLayoutConfig.h"
#include "robo_miner_gui/layout/RoboMinerLayout.h"

using namespace std::placeholders;

ErrorCode RoboMinerLayoutInitHelper::init(
    const RoboMinerLayoutConfig &cfg,
    const RoboMinerLayoutOutInterface &outInterface,
    RoboCommonLayoutInterface &commonInterface, //out param
    RoboMinerLayout &layout) {
  RoboCommonLayoutOutInterface commonOutInterface;
  commonOutInterface.collisionWatcher = outInterface.collisionWatcher;
  commonOutInterface.finishRobotActCb = outInterface.finishRobotActCb;
  commonOutInterface.playerDamageCb = std::bind(
      &PanelHandler::modifyHealthIndicator, &layout._panelHandler, _1);
  commonOutInterface.shutdownGameCb = outInterface.shutdownGameCb;
  commonOutInterface.takeScreenshotCb = outInterface.takeScreenshotCb;

  if (ErrorCode::SUCCESS != layout._commonLayout.init(cfg.commonLayoutCfg,
          commonOutInterface, commonInterface)) {
    LOGERR("_commonLayout.init() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initPanelHandler(cfg.panelHandlerCfg, outInterface,
          commonInterface, layout)) {
    LOGERR("initPanelHandler() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initCrystalHandler(cfg, commonInterface, layout)) {
    LOGERR("initCrystals() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode RoboMinerLayoutInitHelper::initPanelHandler(
    const PanelHandlerConfig &cfg,
    const RoboMinerLayoutOutInterface &outInterface,
    const RoboCommonLayoutInterface &commonInterface, RoboMinerLayout &layout) {
  PanelHandlerOutInterface panelHandlerOutInterface;
  panelHandlerOutInterface.startGameWonAnimCb =
      commonInterface.startGameWonAnimCb;
  panelHandlerOutInterface.startGameLostAnimCb =
      commonInterface.startGameLostAnimCb;
  panelHandlerOutInterface.shutdownControllerCb =
      outInterface.shutdownControllerCb;
  panelHandlerOutInterface.startAchievementWonAnimCb =
      commonInterface.startAchievementWonAnimCb;
  panelHandlerOutInterface.fieldMapRevelealedCb =
      outInterface.fieldMapRevelealedCb;

  if (ErrorCode::SUCCESS != layout._panelHandler.init(cfg,
          panelHandlerOutInterface)) {
    LOGERR("Error in _panel.init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode RoboMinerLayoutInitHelper::initCrystalHandler(
    const RoboMinerLayoutConfig &layoutCfg,
    const RoboCommonLayoutInterface &commonInterface, RoboMinerLayout &layout) {
  CrystalHandlerConfig cfg;
  cfg.crystalRsrcId = layoutCfg.crystalRsrcId;
  cfg.getFieldDescriptionCb = commonInterface.getFieldDescriptionCb;

  if (ErrorCode::SUCCESS != layout._crystalHandler.init(cfg)) {
    LOGERR("Error in _crystalHandler.init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

