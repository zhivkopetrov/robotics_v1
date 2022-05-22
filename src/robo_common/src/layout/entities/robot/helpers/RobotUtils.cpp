//Corresponding header
#include "robo_common/layout/entities/robot/helpers/RobotUtils.h"

//System headers
#include <array>

//Other libraries headers
#include "robo_common/layout/field/FieldUtils.h"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/Log.h"

//Own components headers

Direction RobotUtils::getDirAfterRotation(Direction currDir,
                                          RotationDir rotDir) {
  switch (currDir) {
  case Direction::UP:
    return (RotationDir::LEFT == rotDir) ? Direction::LEFT : Direction::RIGHT;
  case Direction::RIGHT:
    return (RotationDir::LEFT == rotDir) ? Direction::UP : Direction::DOWN;
  case Direction::DOWN:
    return (RotationDir::LEFT == rotDir) ? Direction::RIGHT : Direction::LEFT;
  case Direction::LEFT:
    return (RotationDir::LEFT == rotDir) ? Direction::DOWN : Direction::UP;

  default:
    LOGERR("Received unsupported dir: %d", getEnumValue(currDir));
    return Direction::UP;
  }
}

double RobotUtils::getRotationDegFromDir(Direction dir) {
  switch (dir) {
  case Direction::UP:
    return 0;
  case Direction::RIGHT:
    return 90;
  case Direction::DOWN:
    return 180;
  case Direction::LEFT:
    return 270;

  default:
    LOGERR("Received unsupported dir: %d", getEnumValue(dir));
    return 0;
  }
}

SurroundingTiles RobotUtils::getSurroundingTiles(const FieldDescription &descr,
                                                 const RobotState &state) {
  SurroundingTiles result;

  const auto leftDir = getDirAfterRotation(state.dir, RotationDir::LEFT);
  const auto rightDir = getDirAfterRotation(state.dir, RotationDir::RIGHT);
  constexpr auto posesCtn = 3;
  const std::array<FieldPos, posesCtn> futurePoses { FieldUtils::getAdjacentPos(
      leftDir, state.fieldPos), FieldUtils::getAdjacentPos(state.dir,
      state.fieldPos), FieldUtils::getAdjacentPos(rightDir, state.fieldPos), };

  for (auto i = 0; i < posesCtn; ++i) {
    const auto &pos = futurePoses[i];
    result[i] =
        FieldUtils::isInsideField(pos, descr) ?
            descr.data[pos.row][pos.col] :
            RoboCommonDefines::FIELD_OUT_OF_BOULD_MARKER;
  }

  return result;
}

