//Corresponding header
#include "robo_collector_gui/layout/entities/robot/RobotAI.h"

//C system headers

//C++ system headers
#include <array>

//Other libraries headers
#include "robo_common/layout/entities/robot/helpers/RobotActInterface.h"
#include "robo_common/layout/field/FieldUtils.h"
#include "utils/rng/Rng.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t RobotAI::init(const RobotAIConfig &cfg) {
  if (nullptr == cfg.getFieldDataCb) {
    LOGERR("Error, nullptr provided for RobotAIConfig getFieldDataCb");
    return FAILURE;
  }
  _getFieldDataCb = cfg.getFieldDataCb;

  _fieldEnemyMarker = cfg.fieldEnemyMarker;

  return SUCCESS;
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

  const auto currFieldPos = actInterface.getFieldPosCb();
  const auto currDir = actInterface.getDirCb();

  while (true) {
    const auto moveIdx = rng.getRandomNumber(0, lastMoveIdx);
    const auto selectedMoveType = moves[moveIdx];
    if (MoveType::FORWARD == selectedMoveType) {
      if (isForwardDirValid(currFieldPos, currDir)) {
        actInterface.actCb(selectedMoveType);
        break;
      }
      continue; //try again
    }

    actInterface.actCb(selectedMoveType);
    break;
  }
}

bool RobotAI::isForwardDirValid(const FieldPos &currFieldPos,
                                Direction currDir) const {
  const auto &fieldData = _getFieldDataCb();
  const auto futurePos = FieldUtils::getAdjacentPos(currDir, currFieldPos);
  if (!FieldUtils::isInsideField(futurePos)) {
    return false;
  }

  const auto chosenTile = fieldData[futurePos.row][futurePos.col];
  if (_fieldEnemyMarker != chosenTile) {
    return true;
  }

  return false;
}
