//Corresponding header
#include "robo_collector_gui/RoboCollectorGui.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "sdl_utils/input/InputEvent.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/config/RoboCollectorGuiConfig.h"

using namespace std::placeholders;

int32_t RoboCollectorGui::init(const std::any &cfg) {
  int32_t err = SUCCESS;
  const auto parsedCfg = [&cfg, &err]() {
    RoboCollectorGuiConfig localCfg;
    try {
      localCfg = std::any_cast<const RoboCollectorGuiConfig&>(cfg);
    } catch (const std::bad_any_cast &e) {
      LOGERR("std::any_cast<GuiConfig&> failed, %s", e.what());
      err = FAILURE;
    }
    return localCfg;
  }();
  if (SUCCESS != err) {
    LOGERR("Error, parsing config failed");
    return FAILURE;
  }

  _map.create(parsedCfg.mapRsrcId);

  if (SUCCESS != _field.init(parsedCfg.fieldCfg)) {
    LOGERR("Error in _field.init()");
    return FAILURE;
  }

  if (SUCCESS != initPanelHandler(parsedCfg.panelHandlerCfg)) {
    LOGERR("initPanelHandler() failed");
    return FAILURE;
  }

  if (SUCCESS != initRobots(parsedCfg)) {
    LOGERR("initRobots() failed");
    return FAILURE;
  }

  if (SUCCESS != initCoinHandler(parsedCfg.coinHandlerCfg)) {
    LOGERR("initCoinHandler() failed");
    return FAILURE;
  }

  if (SUCCESS != initController(parsedCfg.controllerCfg)) {
    LOGERR("initController() failed");
    return FAILURE;
  }

  if (SUCCESS != initTurnHelper(parsedCfg)) {
    LOGERR("initTurnHelper() failed");
    return FAILURE;
  }

  if (SUCCESS != _gameEndHelper.init()) {
    LOGERR("_gameEndHelper.init() failed");
    return FAILURE;
  }

  return SUCCESS;
}

void RoboCollectorGui::deinit() {
  for (auto &robot : _robots) {
    robot.deinit();
  }

  _coinHandler.deinit();
}

void RoboCollectorGui::draw() const {
  _map.draw();
  _field.draw();
  _panelHandler.draw();
  _coinHandler.draw();
  _controller.draw();
  for (const auto &robot : _robots) {
    robot.draw();
  }

  _gameEndHelper.draw();
}

void RoboCollectorGui::handleEvent(const InputEvent &e) {
  _controller.handleEvent(e);
}

void RoboCollectorGui::process() {
  _collisionWatcher.process();
}

int32_t RoboCollectorGui::initRobots(const RoboCollectorGuiConfig &guiCfg) {
  const auto& baseCfg = guiCfg.robotBaseCfg;
  RobotOutInterface outInterface;

  outInterface.collisionWatcher = &_collisionWatcher;
  outInterface.playerDamageCb = std::bind(
      &PanelHandler::decreaseHealthIndicator, &_panelHandler, _1);
  outInterface.setFieldDataMarkerCb = std::bind(&Field::setFieldDataMarker,
      &_field, _1, _2);
  outInterface.resetFieldDataMarkerCb = std::bind(&Field::resetFieldDataMarker,
      &_field, _1);
  outInterface.getFieldDataCb = std::bind(&Field::getFieldData, &_field);
  outInterface.finishRobotActCb = std::bind(&TurnHelper::onRobotFinishAct,
      &_turnHelper, _1);

  const std::array<FieldPos, Defines::ROBOTS_CTN> robotsFieldPos { FieldPos(
      guiCfg.fieldCfg.rows - 1, guiCfg.fieldCfg.cols - 1), FieldPos(
      guiCfg.fieldCfg.rows - 1, 0), FieldPos(0, 0), FieldPos(0,
      guiCfg.fieldCfg.cols - 1) };

  const std::array<Direction, Defines::ROBOTS_CTN> robotsInitialDirs {
      Direction::UP, Direction::UP, Direction::DOWN, Direction::DOWN };

  RobotConfig cfg;
  RobotAnimatorConfigBase animatorCfgBase;
  animatorCfgBase.damageMarkerRsrcId = baseCfg.damageMarkerRsrcId;
  for (auto i = 0; i < Defines::ROBOTS_CTN; ++i) {
    if (Defines::PLAYER_ROBOT_IDX == i) {
      animatorCfgBase.robotRsrcId = baseCfg.playerRsrcId;
      animatorCfgBase.frameId = 0;
      cfg.fieldMarker = guiCfg.playerFieldMarker;
      cfg.enemyFieldMarker = guiCfg.enemyFieldMarker;
    } else {
      animatorCfgBase.robotRsrcId = baseCfg.enemiesRsrcId;
      animatorCfgBase.frameId = i - 1;
      cfg.fieldMarker = guiCfg.enemyFieldMarker;
      cfg.enemyFieldMarker = guiCfg.playerFieldMarker;
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
    animatorCfgBase.robotDamageAnimTimerId =
        baseCfg.robotDamageAnimStartTimerId + i;

    if (SUCCESS != _robots[i].init(cfg, animatorCfgBase, outInterface)) {
      LOGERR("Error in _robots[%d].init()", i);
      return FAILURE;
    }
  }

  return SUCCESS;
}

int32_t RoboCollectorGui::initPanelHandler(const PanelHandlerConfig &cfg) {
  PanelHandlerOutInterface outInterface;
  outInterface.gameWonCb = std::bind(&GameEndHelper::gameWon, &_gameEndHelper);
  outInterface.gameLostCb = std::bind(&GameEndHelper::gameLost,
      &_gameEndHelper);

  if (SUCCESS != _panelHandler.init(cfg, outInterface)) {
    LOGERR("Error in _panel.init()");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboCollectorGui::initCoinHandler(const CoinHandlerConfig &cfg) {
  CoinOutInterface outInterface;
  outInterface.collisionWatcher = &_collisionWatcher;
  outInterface.setFieldDataMarkerCb = std::bind(&Field::setFieldDataMarker,
      &_field, _1, _2);
  outInterface.resetFieldDataMarkerCb = std::bind(&Field::resetFieldDataMarker,
      &_field, _1);
  outInterface.getFieldDataCb = std::bind(&Field::getFieldData, &_field);
  outInterface.incrCollectedCoinsCb = std::bind(
      &PanelHandler::increaseCollectedCoins, &_panelHandler, _1);

  if (SUCCESS != _coinHandler.init(cfg, outInterface)) {
    LOGERR("Error in _coinHandler.init()");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboCollectorGui::initController(
    const RoboCollectorControllerConfig &cfg) {
  RoboCollectorControllerOutInterface outInterface;

  if (_robots.empty()) {
    LOGERR("Error, robots array is empty!");
    return FAILURE;
  }
  outInterface.robotActCb = std::bind(&Robot::act,
      &_robots[Defines::PLAYER_ROBOT_IDX], _1);

  if (SUCCESS != _controller.init(cfg, outInterface)) {
    LOGERR("Error in _controller.init()");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboCollectorGui::initTurnHelper(const RoboCollectorGuiConfig &guiCfg) {
  TurnHelperConfig cfg;
  cfg.enablePlayerInputCb = std::bind(&RoboCollectorController::unlockInput,
      &_controller);
  cfg.getFieldDataCb = std::bind(&Field::getFieldData, &_field);
  cfg.fieldEmptyDataMarker = guiCfg.fieldCfg.emptyTileMarker;
  cfg.playerDataMarker = guiCfg.playerFieldMarker;
  cfg.maxRobots = Defines::ROBOTS_CTN;
  for (auto i = 0; i < Defines::ROBOTS_CTN; ++i) {
    cfg.robotActInterfaces.emplace_back(std::bind(&Robot::act, &_robots[i], _1),
        std::bind(&Robot::getFieldPos, &_robots[i]),
        std::bind(&Robot::getDirection, &_robots[i]));
  }

  if (SUCCESS != _turnHelper.init(cfg)) {
    LOGERR("Error in _turnHelper.init()");
    return FAILURE;
  }

  return SUCCESS;
}

