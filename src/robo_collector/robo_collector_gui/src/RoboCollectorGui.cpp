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

  if (SUCCESS != _panelHandler.init(parsedCfg.panelHandlerConfig)) {
    LOGERR("Error in _panel.init()");
    return FAILURE;
  }

  if (SUCCESS != initRobots(parsedCfg)) {
    LOGERR("initRobots() failed");
    return FAILURE;
  }

  if (SUCCESS != initCoinHandler(parsedCfg)) {
    LOGERR("initCoinHandler() failed");
    return FAILURE;
  }

  if (SUCCESS != initController(parsedCfg)) {
    LOGERR("initController() failed");
    return FAILURE;
  }

  if (SUCCESS != initTurnHelper(parsedCfg)) {
    LOGERR("initTurnHelper() failed");
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
}

void RoboCollectorGui::handleEvent(const InputEvent &e) {
  _controller.handleEvent(e);
}

void RoboCollectorGui::process() {
  _collisionWatcher.process();
}

int32_t RoboCollectorGui::initRobots(const RoboCollectorGuiConfig &cfg) {
  RobotCfg robotCfg;
  robotCfg.collisionWatcher = &_collisionWatcher;
  robotCfg.playerDamageCb =
      std::bind(&PanelHandler::decreaseHealthIndicator, &_panelHandler, _1);
  robotCfg.setFieldDataMarkerCb =
      std::bind(&Field::setFieldDataMarker, &_field, _1, _2);
  robotCfg.resetFieldDataMarkerCb =
      std::bind(&Field::resetFieldDataMarker, &_field, _1);
  robotCfg.getFieldDataCb = std::bind(&Field::getFieldData, &_field);
  robotCfg.finishRobotActCb =
      std::bind(&TurnHelper::onRobotFinishAct, &_turnHelper, _1);

  const std::array<FieldPos, Defines::ROBOTS_CTN> robotsFieldPos {
    FieldPos(cfg.fieldCfg.rows - 1, cfg.fieldCfg.cols - 1),
    FieldPos(cfg.fieldCfg.rows - 1, 0),
    FieldPos(0, 0),
    FieldPos(0, cfg.fieldCfg.cols - 1)
  };

  const std::array<Direction, Defines::ROBOTS_CTN> robotsInitialDirs {
    Direction::UP, Direction::UP, Direction::DOWN, Direction::DOWN
  };

  for (auto i = 0; i < Defines::ROBOTS_CTN; ++i) {
    if (Defines::PLAYER_ROBOT_IDX == i) {
      robotCfg.rsrcId = cfg.robotBlinkyRsrcId;
      robotCfg.frameId = 0;
      robotCfg.fieldMarker = cfg.playerFieldMarker;
      robotCfg.enemyFieldMarker = cfg.enemyFieldMarker;
    } else {
      robotCfg.rsrcId = cfg.robotEnemiesRsrcId;
      robotCfg.frameId = i - 1;
      robotCfg.fieldMarker = cfg.enemyFieldMarker;
      robotCfg.enemyFieldMarker = cfg.playerFieldMarker;
    }
    robotCfg.robotId = i;
    robotCfg.fieldPos = robotsFieldPos[i];
    robotCfg.initialDir = robotsInitialDirs[i];
    robotCfg.moveAnimTimerId = cfg.robotsMoveAnimStartTimerId + i;
    robotCfg.wallCollisionAnimTimerId =
        cfg.robotsWallCollisionAnimStartTimerId + i;

    if (SUCCESS != _robots[i].init(robotCfg)) {
      LOGERR("Error in _robots[%d].init()", i);
      return FAILURE;
    }
  }

  return SUCCESS;
}

int32_t RoboCollectorGui::initCoinHandler(const RoboCollectorGuiConfig &cfg) {
  CoinHandlerConfig coinHandlerCfg;
  coinHandlerCfg.collisionWatcher = &_collisionWatcher;
  coinHandlerCfg.animRsrcIds = cfg.coinAnimRsrcIds;
  coinHandlerCfg.fieldDataMarkers = cfg.coinFieldDataMarkers;
  coinHandlerCfg.fieldEmptyDataMarker = cfg.fieldCfg.emptyTileMarker;
  coinHandlerCfg.maxCoins = cfg.maxCoins;
  coinHandlerCfg.rotateAnimFirstTimerId = cfg.coinRotateAnimFirstTimerId;
  coinHandlerCfg.collectAnimFirstTimerId = cfg.coinCollectAnimFirstTimerId;
  coinHandlerCfg.respawnAnimFirstTimerId = cfg.coinRespawnAnimFirstTimerId;
  coinHandlerCfg.setFieldDataMarkerCb =
      std::bind(&Field::setFieldDataMarker, &_field, _1, _2);
  coinHandlerCfg.resetFieldDataMarkerCb =
      std::bind(&Field::resetFieldDataMarker, &_field, _1);
  coinHandlerCfg.getFieldDataCb = std::bind(&Field::getFieldData, &_field);
  coinHandlerCfg.incrCollectedCoinsCb =
      std::bind(&PanelHandler::increaseCollectedCoins, &_panelHandler, _1);

  if (SUCCESS != _coinHandler.init(coinHandlerCfg)) {
    LOGERR("Error in _coinHandler.init()");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboCollectorGui::initController(const RoboCollectorGuiConfig& cfg) {
  RoboCollectorControllerConfig collectorCfg;
  if (_robots.empty()) {
    LOGERR("Error, robots array is empty!");
    return FAILURE;
  }
  collectorCfg.robotActCb =
      std::bind(&Robot::act, &_robots[Defines::PLAYER_ROBOT_IDX], _1);
  collectorCfg.moveButtonsRsrcIds = cfg.moveButtonsRsrcIds;
  collectorCfg.maxMoveButtons = cfg.maxMoveButtons;
  collectorCfg.moveButtonInfoTextFontId = cfg.moveButtonsInfoTextFontId;
  collectorCfg.horDelimiterRsrcId = cfg.horDelimiterRsrcId;
  collectorCfg.vertDelimiterRsrcId = cfg.vertDelimiterRsrcId;

  if (SUCCESS != _controller.init(collectorCfg)) {
    LOGERR("Error in _controller.init()");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboCollectorGui::initTurnHelper(const RoboCollectorGuiConfig& cfg) {
  TurnHelperConfig turnHelperCfg;
  turnHelperCfg.enablePlayerInputCb =
      std::bind(&RoboCollectorController::unlockInput, &_controller);
  turnHelperCfg.getFieldDataCb = std::bind(&Field::getFieldData, &_field);
  turnHelperCfg.fieldEmptyDataMarker = cfg.fieldCfg.emptyTileMarker;
  turnHelperCfg.playerDataMarker = cfg.playerFieldMarker;
  turnHelperCfg.maxRobots = Defines::ROBOTS_CTN;
  for (auto i = 0; i < Defines::ROBOTS_CTN; ++i) {
    turnHelperCfg.robotActInterfaces.emplace_back(
        std::bind(&Robot::act, &_robots[i], _1),
        std::bind(&Robot::getFieldPos, &_robots[i]),
        std::bind(&Robot::getDirection, &_robots[i]));
  }

  if (SUCCESS != _turnHelper.init(turnHelperCfg)) {
    LOGERR("Error in _turnHelper.init()");
    return FAILURE;
  }

  return SUCCESS;
}

