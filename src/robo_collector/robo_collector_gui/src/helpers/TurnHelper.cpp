//Corresponding header
#include "robo_collector_gui/helpers/TurnHelper.h"

//System headers

//Other libraries headers
#include "utils/log/Log.h"

//Own components headers

ErrorCode TurnHelper::init(const TurnHelperConfig &cfg) {
  RobotAIConfig robotAiCfg;
  robotAiCfg.getFieldDescriptionCb = cfg.getFieldDescriptionCb;
  robotAiCfg.fieldEnemyMarker = cfg.fieldEnemyMarker;
  if (ErrorCode::SUCCESS != _robotAI.init(robotAiCfg)) {
    LOGERR("Error, robotAI.init() failed");
    return ErrorCode::FAILURE;
  }

  if (nullptr == cfg.enablePlayerInputCb) {
    LOGERR("Error, nullptr provided for enablePlayerInputCb");
    return ErrorCode::FAILURE;
  }
  _enablePlayerInputCb = cfg.enablePlayerInputCb;

  _maxRobots = cfg.maxRobots;
  _robotActInterfaces = cfg.robotActInterfaces;

  return ErrorCode::SUCCESS;
}

void TurnHelper::onRobotFinishAct(const RobotState& state,
                                  [[maybe_unused]]MoveOutcome moveOutcome) {
  const auto lastRobotIdx = _maxRobots - 1;
  if (lastRobotIdx == state.robotId) {
    _activeRobotId = RoboCommonDefines::PLAYER_ROBOT_IDX;
    _enablePlayerInputCb();
  } else {
    _activeRobotId = state.robotId + 1;
    _robotAI.makeMove(_robotActInterfaces[_activeRobotId]);
  }
}

bool TurnHelper::isPlayerTurnActive() const {
  return RoboCommonDefines::PLAYER_ROBOT_IDX == _activeRobotId;
}

