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
  ++_currTurn;
  const auto lastRobotIdx = _maxRobots - 1;
  if (lastRobotIdx == robotId) {
    _activeRobotId = Defines::PLAYER_ROBOT_IDX;
    _enablePlayerInputCb();
  } else {
    _activeRobotId = robotId + 1;
    _robotAI.makeMove(_robotActInterfaces[_activeRobotId]);
  }
}

