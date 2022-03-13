//Corresponding header
#include "robo_collector_gui/layout/entities/robot/RobotAI.h"

//System headers
#include <array>

//Other libraries headers
#include "robo_common/layout/entities/robot/helpers/RobotActInterface.h"
#include "robo_common/layout/field/FieldUtils.h"
#include "utils/rng/Rng.h"
#include "utils/Log.h"

//Own components headers

ErrorCode RobotAI::init(const RobotAIConfig &cfg) {
  if (nullptr == cfg.getFieldDescriptionCb) {
    LOGERR("Error, nullptr provided for RobotAIConfig GetFieldDescriptionCb");
    return ErrorCode::FAILURE;
  }
  _getFieldDescriptionCb = cfg.getFieldDescriptionCb;

  _fieldEnemyMarker = cfg.fieldEnemyMarker;
  return ErrorCode::SUCCESS;
}

//TODO improve AI
//don't rotate towards the edge of the map
void RobotAI::makeMove(const RobotActInterface& actInterface) {
  auto &rng = Rng::getInstance();
  constexpr auto movesCount = 5;
  constexpr auto lastMoveIdx = movesCount - 1;
  //have bigger possibility to move, as opposed to rotation
  constexpr std::array<MoveType, movesCount> moves { MoveType::FORWARD,
      MoveType::FORWARD, MoveType::FORWARD, MoveType::ROTATE_LEFT,
      MoveType::ROTATE_RIGHT };

  while (true) {
    const auto moveIdx = rng.getRandomNumber(0, lastMoveIdx);
    const auto selectedMoveType = moves[moveIdx];
    if (MoveType::FORWARD == selectedMoveType) {
      if (isForwardDirValid(actInterface.getRobotStateCb())) {
        actInterface.actCb(selectedMoveType);
        break;
      }
      continue; //try again
    }

    actInterface.actCb(selectedMoveType);
    break;
  }
}

bool RobotAI::isForwardDirValid(const RobotState& state) const {
  const auto &fieldDescr = _getFieldDescriptionCb();
  const auto futurePos = FieldUtils::getAdjacentPos(state.dir, state.fieldPos);
  if (!FieldUtils::isInsideField(futurePos, fieldDescr)) {
    return false;
  }

  const auto chosenTile = fieldDescr.data[futurePos.row][futurePos.col];
  if (_fieldEnemyMarker != chosenTile) {
    return true;
  }

  return false;
}
