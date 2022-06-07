//Corresponding header
#include "robo_cleaner_common/message_helpers/RoboCleanerMessageHelpers.h"

//System headers

//Other libraries headers
#include "robo_cleaner_interfaces/msg/robot_move_type.hpp"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/Log.h"

//Own components headers

using robo_cleaner_interfaces::msg::RobotMoveType;

int8_t getMoveTypeField(MoveType moveType) {
  switch (moveType) {
  case MoveType::FORWARD:
    return RobotMoveType::FORWARD;
  case MoveType::ROTATE_LEFT:
    return RobotMoveType::ROTATE_LEFT;
  case MoveType::ROTATE_RIGHT:
    return RobotMoveType::ROTATE_RIGHT;
  default:
    LOGERR("Error, received unsupported MoveType: %d", getEnumValue(moveType));
    return RobotMoveType::FORWARD;
  }
}

MoveType getMoveType(int8_t moveType) {
  switch (moveType) {
  case RobotMoveType::FORWARD:
    return MoveType::FORWARD;
  case RobotMoveType::ROTATE_LEFT:
    return MoveType::ROTATE_LEFT;
  case RobotMoveType::ROTATE_RIGHT:
    return MoveType::ROTATE_RIGHT;
  default:
    LOGERR("Error, received unsupported RobotMoveType: %hhu", moveType);
    return MoveType::UNKNOWN;
  }
}
