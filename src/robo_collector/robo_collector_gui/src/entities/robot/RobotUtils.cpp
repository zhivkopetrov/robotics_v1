//Corresponding header
#include "robo_collector_gui/entities/robot/RobotUtils.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/data_type/EnumClassUtils.h"
#include "utils/Log.h"

//Own components headers

Direction RobotUtils::getDirAfterRotation(Direction currDir,
                                          bool isLeftRotation) {
  switch (currDir) {
  case Direction::UP:
    return isLeftRotation ? Direction::LEFT : Direction::RIGHT;
  case Direction::RIGHT:
    return isLeftRotation ? Direction::UP : Direction::DOWN;
  case Direction::DOWN:
    return isLeftRotation ? Direction::RIGHT : Direction::LEFT;
  case Direction::LEFT:
    return isLeftRotation ? Direction::DOWN : Direction::UP;

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
