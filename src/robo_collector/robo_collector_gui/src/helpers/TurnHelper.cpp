//Corresponding header
#include "robo_collector_gui/helpers/TurnHelper.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t TurnHelper::init(const TurnHelperConfig& cfg) {
  RobotAIConfig robotAiCfg;
  robotAiCfg.getFieldDataCb = cfg.getFieldDataCb;
  robotAiCfg.fieldEmptyDataMarker = cfg.fieldEmptyDataMarker;
  robotAiCfg.playerDataMarker = cfg.playerDataMarker;
  if (SUCCESS != _robotAI.init(robotAiCfg)) {
    LOGERR("Error, robotAI.init() failed");
    return FAILURE;
  }

  if (nullptr == cfg.enablePlayerInputCb) {
    LOGERR("Error, nullptr provided for enablePlayerInputCb");
    return FAILURE;
  }
  _enablePlayerInputCb = cfg.enablePlayerInputCb;

  _maxRobots = cfg.maxRobots;
  _robotActInterfaces = cfg.robotActInterfaces;

  return SUCCESS;
}

void TurnHelper::onRobotFinishAct(int32_t robotId) {
  LOGC("======= FINISH TURN: %d for robotId: %d", _currTurn, robotId);
  ++_currTurn;

  const auto lastRobotIdx = _maxRobots - 1;
  if (lastRobotIdx == robotId) {
    _activeRobotId = Defines::PLAYER_ROBOT_IDX;
    LOGY("======= START TURN: %d for robotId: %d", _currTurn, _activeRobotId);
    _enablePlayerInputCb();
  } else {
    _activeRobotId = robotId + 1;
    LOGY("======= START TURN: %d for robotId: %d", _currTurn, _activeRobotId);
    _robotAI.makeMove(_robotActInterfaces[_activeRobotId]);
  }
}

