//Corresponding header
#include "robo_collector_gui/layout/helpers/RoboCollectorLayoutInitHelper.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/layout/helpers/RoboCollectorLayoutInterfaces.h"
#include "robo_collector_gui/layout/config/RoboCollectorLayoutConfig.h"
#include "robo_collector_gui/layout/RoboCollectorLayout.h"

using namespace std::placeholders;

int32_t RoboCollectorLayoutInitHelper::init(
    const RoboCollectorLayoutConfig &cfg,
    const RoboCollectorLayoutOutInterface &outInterface,
    RoboCommonLayoutInterface &commonInterface,
    RoboCollectorLayout &layout) {
  RoboCommonLayoutOutInterface commonOutInterface;
  commonOutInterface.collisionWatcher = outInterface.collisionWatcher;
  commonOutInterface.finishRobotActCb = outInterface.finishRobotActCb;
  commonOutInterface.playerDamageCb = std::bind(
      &PanelHandler::decreaseHealthIndicator, &layout._panelHandler, _1);

  if (SUCCESS != layout._commonLayout.init(
      cfg.commonLayoutCfg, commonOutInterface, commonInterface)) {
    LOGERR("_commonLayout.init() failed");
    return FAILURE;
  }

  if (SUCCESS !=
      initPanelHandler(cfg.panelHandlerCfg, commonInterface, layout)) {
    LOGERR("initPanelHandler() failed");
    return FAILURE;
  }

  if (SUCCESS != initRobots(cfg, outInterface, commonInterface, layout)) {
    LOGERR("initRobots() failed");
    return FAILURE;
  }

  if (SUCCESS != initCoinHandler(
      cfg.coinHandlerCfg, outInterface, commonInterface, layout)) {
    LOGERR("initCoinHandler() failed");
    return FAILURE;
  }

  if (SUCCESS != initController(cfg.controllerCfg, commonInterface, layout)) {
    LOGERR("initController() failed");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboCollectorLayoutInitHelper::initRobots(
    const RoboCollectorLayoutConfig &layoutCfg,
    const RoboCollectorLayoutOutInterface &outInterface,
    const RoboCommonLayoutInterface& commonInterface,
    RoboCollectorLayout &layout) {
  const auto &baseCfg = layoutCfg.commonLayoutCfg.robotBaseCfg;
  RobotOutInterface robotOutInterface;

  robotOutInterface.collisionWatcher = outInterface.collisionWatcher;
  robotOutInterface.playerDamageCb = std::bind(
      &PanelHandler::decreaseHealthIndicator, &layout._panelHandler, _1);
  robotOutInterface.setFieldDataMarkerCb = commonInterface.setFieldDataMarkerCb;
  robotOutInterface.resetFieldDataMarkerCb =
      commonInterface.resetFieldDataMarkerCb;
  robotOutInterface.getFieldDataCb = commonInterface.getFieldDataCb;
  robotOutInterface.finishRobotActCb = outInterface.finishRobotActCb;

  const auto &fieldCfg = layoutCfg.commonLayoutCfg.fieldCfg;
  const std::array<FieldPos, Defines::ENEMIES_CTN> robotsFieldPos { FieldPos(
      fieldCfg.rows - 1, 0), FieldPos(0, 0), FieldPos(0, fieldCfg.cols - 1) };

  const std::array<Direction, Defines::ENEMIES_CTN> robotsInitialDirs {
      Direction::UP, Direction::DOWN, Direction::DOWN };

  RobotConfig cfg;
  RobotAnimatorConfigBase animatorCfgBase;
  animatorCfgBase.damageMarkerRsrcId = baseCfg.damageMarkerRsrcId;
  constexpr auto playerIdOffset = 1; //+1, because Player robot has index 0
  for (auto i = 0; i < Defines::ENEMIES_CTN; ++i) {
    animatorCfgBase.robotRsrcId = baseCfg.enemiesRsrcId;
    animatorCfgBase.frameId = i;
    cfg.fieldMarker = layoutCfg.commonLayoutCfg.enemyFieldMarker;
    cfg.enemyFieldMarker = layoutCfg.commonLayoutCfg.playerFieldMarker;
    cfg.robotId = i + playerIdOffset;
    animatorCfgBase.robotId = i + playerIdOffset;
    cfg.fieldPos = robotsFieldPos[i];
    cfg.dir = robotsInitialDirs[i];
    animatorCfgBase.moveAnimTimerId =
        baseCfg.moveAnimStartTimerId + i + playerIdOffset;
    animatorCfgBase.wallCollisionAnimTimerId =
        baseCfg.wallCollisionAnimStartTimerId + i + playerIdOffset;
    animatorCfgBase.robotCollisionAnimTimerId =
        baseCfg.robotCollisionAnimStartTimerId + i + playerIdOffset;
    animatorCfgBase.robotDamageAnimTimerId =
        baseCfg.robotDamageAnimStartTimerId + i + playerIdOffset;

    if (SUCCESS !=
        layout._enemyRobots[i].init(cfg, animatorCfgBase, robotOutInterface)) {
      LOGERR("Error in _enemyRobots[%d].init()", i);
      return FAILURE;
    }
  }

  return SUCCESS;
}

int32_t RoboCollectorLayoutInitHelper::initPanelHandler(
    const PanelHandlerConfig &cfg,
    RoboCommonLayoutInterface &commonInterface,
    RoboCollectorLayout &layout) {
  PanelHandlerOutInterface outInterface;
  outInterface.gameWonCb = commonInterface.gameWonCb;
  outInterface.gameLostCb = commonInterface.gameLostCb;

  if (SUCCESS != layout._panelHandler.init(cfg, outInterface)) {
    LOGERR("Error in _panel.init()");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboCollectorLayoutInitHelper::initCoinHandler(
    const CoinHandlerConfig &cfg,
    const RoboCollectorLayoutOutInterface &interface,
    const RoboCommonLayoutInterface& commonInterface,
    RoboCollectorLayout &layout) {
  CoinOutInterface outInterface;
  outInterface.collisionWatcher = interface.collisionWatcher;
  outInterface.setFieldDataMarkerCb = commonInterface.setFieldDataMarkerCb;
  outInterface.resetFieldDataMarkerCb = commonInterface.resetFieldDataMarkerCb;
  outInterface.getFieldDataCb = commonInterface.getFieldDataCb;
  outInterface.incrCollectedCoinsCb = std::bind(
      &PanelHandler::increaseCollectedCoins, &layout._panelHandler, _1);
  outInterface.isPlayerTurnActiveCb = interface.isPlayerTurnActiveCb;

  if (SUCCESS != layout._coinHandler.init(cfg, outInterface)) {
    LOGERR("Error in _coinHandler.init()");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboCollectorLayoutInitHelper::initController(
    const RoboCollectorControllerConfig &cfg,
    const RoboCommonLayoutInterface& commonInterface,
    RoboCollectorLayout &layout) {
  RoboCollectorControllerOutInterface outInterface;
  outInterface.robotActCb = commonInterface.playerRobotActInterface.actCb;
  outInterface.helpActivatedCb = std::bind(
      &RoboCollectorLayout::activateHelpPage, &layout);
  outInterface.settingActivatedCb =
      std::bind(&RoboCollectorLayout::settingsActivated, &layout);

  if (SUCCESS != layout._controller.init(cfg, outInterface)) {
    LOGERR("Error in _controller.init()");
    return FAILURE;
  }

  return SUCCESS;
}
