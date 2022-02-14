//Corresponding header
#include "robo_common/layout/entities/robot/RobotUtils.h"

//C system headers

//C++ system headers

//Other libraries headers
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
