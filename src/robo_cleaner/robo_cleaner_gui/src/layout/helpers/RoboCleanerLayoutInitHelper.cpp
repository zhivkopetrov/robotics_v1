//Corresponding header
#include "robo_cleaner_gui/layout/helpers/RoboCleanerLayoutInitHelper.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_cleaner_gui/layout/helpers/RoboCleanerLayoutInterfaces.h"
#include "robo_cleaner_gui/layout/config/RoboCleanerLayoutConfig.h"
#include "robo_cleaner_gui/layout/RoboCleanerLayout.h"

using namespace std::placeholders;

int32_t RoboCleanerLayoutInitHelper::init(
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

  if (SUCCESS != layout._entityHandler.init(cfg.entityHandlerCfg,
      commonInterface.getFieldDescriptionCb)) {
    LOGERR("Error, _entityHandler.init() failed");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboCleanerLayoutInitHelper::initPanelHandler(
    const PanelHandlerConfig &cfg, RoboCommonLayoutInterface &commonInterface,
    RoboCleanerLayout &layout) {
  PanelHandlerOutInterface outInterface;
  outInterface.startGameWonAnimCb = commonInterface.startGameWonAnimCb;
  outInterface.startGameLostAnimCb = commonInterface.startGameLostAnimCb;
  outInterface.startAchievementWonAnimCb =
      commonInterface.startAchievementWonAnimCb;
  outInterface.energyDepletedCb =
      std::bind(&RoboCleanerLayout::onEnergyDepleted, &layout);

  if (SUCCESS != layout._panelHandler.init(cfg, outInterface)) {
    LOGERR("Error in _panel.init()");
    return FAILURE;
  }

  return SUCCESS;
}

