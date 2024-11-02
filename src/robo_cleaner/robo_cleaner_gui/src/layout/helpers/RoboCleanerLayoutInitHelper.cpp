//Corresponding header
#include "robo_cleaner_gui/layout/helpers/RoboCleanerLayoutInitHelper.h"

//System headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/log/Log.h"

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
  commonOutInterface.playerRobotDamageCollisionCb =
      outInterface.playerRobotDamageCollisionCb;
  commonOutInterface.playerDamageCb = std::bind(
      &PanelHandler::modifyHealthIndicator, &layout._panelHandler, _1);
  commonOutInterface.shutdownGameCb = outInterface.shutdownGameCb;
  commonOutInterface.takeScreenshotCb = outInterface.takeScreenshotCb;
  commonOutInterface.objectApproachOverlayTriggeredCb =
      outInterface.objectApproachOverlayTriggeredCb;

  if (ErrorCode::SUCCESS != layout._commonLayout.init(cfg.commonLayoutCfg,
          commonOutInterface, commonInterface)) {
    LOGERR("_commonLayout.init() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initPanelHandler(cfg.panelHandlerCfg,
          commonInterface, outInterface, layout)) {
    LOGERR("initPanelHandler() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initEntityHandler(cfg, outInterface, layout)) {
    LOGERR("initPanelHandler() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode RoboCleanerLayoutInitHelper::initEntityHandler(
    const RoboCleanerLayoutConfig &layoutCfg,
    const RoboCleanerLayoutOutInterface &outInterface,
    RoboCleanerLayout &layout) {
  const EntityHandlerOutInterface entityHandlerOutInterface = {
      .objectApproachOverlayTriggeredCb =
          outInterface.objectApproachOverlayTriggeredCb, .collisionWatcher =
          outInterface.collisionWatcher };

  if (ErrorCode::SUCCESS != layout._entityHandler.init(
          layoutCfg.entityHandlerCfg, entityHandlerOutInterface,
          layoutCfg.commonLayoutCfg.fieldCfg.description)) {
    LOGERR("Error, _entityHandler.init() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode RoboCleanerLayoutInitHelper::initPanelHandler(
    const PanelHandlerConfig &cfg,
    const RoboCommonLayoutInterface &commonInterface,
    const RoboCleanerLayoutOutInterface &outInterface,
    RoboCleanerLayout &layout) {
  PanelHandlerOutInterface panelHandlerOutInterface;
  panelHandlerOutInterface.startGameLostAnimCb =
      commonInterface.startGameLostAnimCb;
  panelHandlerOutInterface.startAchievementWonAnimCb =
      commonInterface.startAchievementWonAnimCb;
  panelHandlerOutInterface.shutdownControllerCb =
      outInterface.shutdownControllerCb;
  panelHandlerOutInterface.fieldMapRevelealedCb =
      outInterface.fieldMapRevelealedCb;
  panelHandlerOutInterface.fieldMapCleanedCb = outInterface.fieldMapCleanedCb;
  panelHandlerOutInterface.energyDepletedCb = [](){}; //do nothing

  if (ErrorCode::SUCCESS != layout._panelHandler.init(cfg,
          panelHandlerOutInterface)) {
    LOGERR("Error in _panel.init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

