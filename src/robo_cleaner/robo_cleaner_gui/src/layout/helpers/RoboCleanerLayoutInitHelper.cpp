//Corresponding header
#include "robo_cleaner_gui/layout/helpers/RoboCleanerLayoutInitHelper.h"

//System headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_cleaner_gui/layout/helpers/RoboCleanerLayoutInterfaces.h"
#include "robo_cleaner_gui/layout/config/RoboCleanerLayoutConfig.h"
#include "robo_cleaner_gui/layout/RoboCleanerLayout.h"

using namespace std::placeholders;

ErrorCode RoboCleanerLayoutInitHelper::init(
    const RoboCleanerLayoutConfig &cfg,
    const RoboCleanerLayoutOutInterface &outInterface,
    RoboCommonLayoutInterface &commonInterface, //out param
    RoboCleanerLayout &layout) {
  RoboCommonLayoutOutInterface commonOutInterface;
  commonOutInterface.collisionWatcher = outInterface.collisionWatcher;
  commonOutInterface.finishRobotActCb = outInterface.finishRobotActCb;
  commonOutInterface.playerDamageCb = std::bind(
      &PanelHandler::decreaseHealthIndicator, &layout._panelHandler, _1);
  commonOutInterface.shutdownGameCb = outInterface.shutdownGameCb;

  if (ErrorCode::SUCCESS != layout._commonLayout.init(cfg.commonLayoutCfg,
          commonOutInterface, commonInterface)) {
    LOGERR("_commonLayout.init() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initPanelHandler(cfg.panelHandlerCfg,
          commonInterface, layout)) {
    LOGERR("initPanelHandler() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != layout._entityHandler.init(cfg.entityHandlerCfg,
          cfg.commonLayoutCfg.fieldCfg.description)) {
    LOGERR("Error, _entityHandler.init() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode RoboCleanerLayoutInitHelper::initPanelHandler(
    const PanelHandlerConfig &cfg, RoboCommonLayoutInterface &commonInterface,
    RoboCleanerLayout &layout) {
  PanelHandlerOutInterface outInterface;
  outInterface.startGameWonAnimCb = commonInterface.startGameWonAnimCb;
  outInterface.startGameLostAnimCb = commonInterface.startGameLostAnimCb;
  outInterface.startAchievementWonAnimCb =
      commonInterface.startAchievementWonAnimCb;
  outInterface.energyDepletedCb = std::bind(
      &RoboCleanerLayout::onEnergyDepleted, &layout);

  if (ErrorCode::SUCCESS != layout._panelHandler.init(cfg, outInterface)) {
    LOGERR("Error in _panel.init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

