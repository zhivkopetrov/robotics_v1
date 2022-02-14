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
    const RoboCollectorLayoutOutInterface &interface,
    RoboCollectorLayout &layout) {
  layout._map.create(cfg.mapRsrcId);

  if (SUCCESS != layout._field.init(cfg.fieldCfg)) {
    LOGERR("Error in _field.init()");
    return FAILURE;
  }

  if (SUCCESS != initPanelHandler(cfg.panelHandlerCfg, layout)) {
    LOGERR("initPanelHandler() failed");
    return FAILURE;
  }

  if (SUCCESS != initRobots(cfg, interface, layout)) {
    LOGERR("initRobots() failed");
    return FAILURE;
  }

  if (SUCCESS != initCoinHandler(cfg.coinHandlerCfg, interface, layout)) {
    LOGERR("initCoinHandler() failed");
    return FAILURE;
  }

  if (SUCCESS != initController(cfg.controllerCfg, layout)) {
    LOGERR("initController() failed");
    return FAILURE;
  }

  if (SUCCESS != layout._gameEndAnimator.init()) {
    LOGERR("_gameEndAnimator.init() failed");
    return FAILURE;
  }

  if (SUCCESS != layout._roboMinerGui.init(cfg.roboMinerGuiCfg)) {
    LOGERR("Error, _roboMinerGui.init() failed");
    return FAILURE;
  }

  if (SUCCESS != layout._roboCleanerGui.init(cfg.roboCleanerGuiCfg)) {
    LOGERR("Error, _roboCleanerGui.init() failed");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboCollectorLayoutInitHelper::initRobots(
    const RoboCollectorLayoutConfig &layoutCfg,
    const RoboCollectorLayoutOutInterface &interface,
    RoboCollectorLayout &layout) {
  const auto &baseCfg = layoutCfg.robotBaseCfg;
  RobotOutInterface outInterface;

  outInterface.collisionWatcher = interface.collisionWatcher;
  outInterface.playerDamageCb = std::bind(
      &PanelHandler::decreaseHealthIndicator, &layout._panelHandler, _1);
  outInterface.setFieldDataMarkerCb = std::bind(&Field::setFieldDataMarker,
      &layout._field, _1, _2);
  outInterface.resetFieldDataMarkerCb = std::bind(&Field::resetFieldDataMarker,
      &layout._field, _1);
  outInterface.getFieldDataCb = std::bind(&Field::getFieldData, &layout._field);
  outInterface.finishRobotActCb = interface.finishRobotActCb;

  const std::array<FieldPos, Defines::ROBOTS_CTN> robotsFieldPos { FieldPos(
      layoutCfg.fieldCfg.rows - 1, layoutCfg.fieldCfg.cols - 1), FieldPos(
      layoutCfg.fieldCfg.rows - 1, 0), FieldPos(0, 0), FieldPos(0,
      layoutCfg.fieldCfg.cols - 1) };

  const std::array<Direction, Defines::ROBOTS_CTN> robotsInitialDirs {
      Direction::UP, Direction::UP, Direction::DOWN, Direction::DOWN };

  RobotConfig cfg;
  RobotAnimatorConfigBase animatorCfgBase;
  animatorCfgBase.damageMarkerRsrcId = baseCfg.damageMarkerRsrcId;
  for (auto i = 0; i < Defines::ROBOTS_CTN; ++i) {
    if (RoboCommonDefines::PLAYER_ROBOT_IDX == i) {
      animatorCfgBase.robotRsrcId = baseCfg.playerRsrcId;
      animatorCfgBase.frameId = 0;
      cfg.fieldMarker = layoutCfg.playerFieldMarker;
      cfg.enemyFieldMarker = layoutCfg.enemyFieldMarker;
    } else {
      animatorCfgBase.robotRsrcId = baseCfg.enemiesRsrcId;
      animatorCfgBase.frameId = i - 1;
      cfg.fieldMarker = layoutCfg.enemyFieldMarker;
      cfg.enemyFieldMarker = layoutCfg.playerFieldMarker;
    }
    cfg.robotId = i;
    animatorCfgBase.robotId = i;
    cfg.fieldPos = robotsFieldPos[i];
    cfg.dir = robotsInitialDirs[i];
    animatorCfgBase.moveAnimTimerId = baseCfg.moveAnimStartTimerId + i;
    animatorCfgBase.wallCollisionAnimTimerId =
        baseCfg.wallCollisionAnimStartTimerId + i;
    animatorCfgBase.robotCollisionAnimTimerId =
        baseCfg.robotCollisionAnimStartTimerId + i;
    animatorCfgBase.robotDamageAnimTimerId = baseCfg.robotDamageAnimStartTimerId
        + i;

    if (SUCCESS != layout._robots[i].init(cfg, animatorCfgBase, outInterface)) {
      LOGERR("Error in _robots[%d].init()", i);
      return FAILURE;
    }
  }

  return SUCCESS;
}

int32_t RoboCollectorLayoutInitHelper::initPanelHandler(
    const PanelHandlerConfig &cfg, RoboCollectorLayout &layout) {
  PanelHandlerOutInterface outInterface;
  outInterface.gameWonCb = std::bind(&GameEndAnimator::gameWon,
      &layout._gameEndAnimator);
  outInterface.gameLostCb = std::bind(&GameEndAnimator::gameLost,
      &layout._gameEndAnimator);

  if (SUCCESS != layout._panelHandler.init(cfg, outInterface)) {
    LOGERR("Error in _panel.init()");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboCollectorLayoutInitHelper::initCoinHandler(
    const CoinHandlerConfig &cfg,
    const RoboCollectorLayoutOutInterface &interface,
    RoboCollectorLayout &layout) {
  CoinOutInterface outInterface;
  outInterface.collisionWatcher = interface.collisionWatcher;
  outInterface.setFieldDataMarkerCb = std::bind(&Field::setFieldDataMarker,
      &layout._field, _1, _2);
  outInterface.resetFieldDataMarkerCb = std::bind(&Field::resetFieldDataMarker,
      &layout._field, _1);
  outInterface.getFieldDataCb = std::bind(&Field::getFieldData, &layout._field);
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
    const RoboCollectorControllerConfig &cfg, RoboCollectorLayout &layout) {
  RoboCollectorControllerOutInterface outInterface;

  if (layout._robots.empty()) {
    LOGERR("Error, robots array is empty!");
    return FAILURE;
  }
  outInterface.robotActCb = std::bind(&Robot::act,
      &layout._robots[RoboCommonDefines::PLAYER_ROBOT_IDX], _1);
  outInterface.helpActivatedCb = std::bind(
      &RoboCollectorLayout::activateHelpPage, &layout);
  outInterface.settingActivatedCb = std::bind(
      &RoboCollectorLayout::changeGameType, &layout, _1);

  if (SUCCESS != layout._controller.init(cfg, outInterface)) {
    LOGERR("Error in _controller.init()");
    return FAILURE;
  }

  return SUCCESS;
}
