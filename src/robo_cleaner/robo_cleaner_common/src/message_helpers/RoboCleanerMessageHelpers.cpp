//Corresponding header
#include "robo_cleaner_common/message_helpers/RoboCleanerMessageHelpers.h"

//System headers

//Other libraries headers
#include "robo_cleaner_interfaces/msg/robot_move_type.hpp"
#include "robo_cleaner_interfaces/msg/initial_robot_state.hpp"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/log/Log.h"

//Own components headers

using robo_cleaner_interfaces::msg::RobotMoveType;
using robo_cleaner_interfaces::msg::InitialRobotState;

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

int32_t getRobotDirectionField(Direction dir) {
  switch (dir) {
  case Direction::UP:
    return InitialRobotState::DIRECTION_UP;
  case Direction::RIGHT:
    return InitialRobotState::DIRECTION_RIGHT;
  case Direction::DOWN:
    return InitialRobotState::DIRECTION_DOWN;
  case Direction::LEFT:
    return InitialRobotState::DIRECTION_LEFT;
  default:
    LOGERR("Error, received unsupported Direction: %d", getEnumValue(dir));
    return InitialRobotState::DIRECTION_UP;
  }
}
Direction getRobotDirection(int32_t dir) {
  switch (dir) {
  case InitialRobotState::DIRECTION_UP:
    return Direction::UP;
  case InitialRobotState::DIRECTION_RIGHT:
    return Direction::RIGHT;
  case InitialRobotState::DIRECTION_DOWN:
    return Direction::DOWN;
  case InitialRobotState::DIRECTION_LEFT:
    return Direction::LEFT;
  default:
    LOGERR("Error, received unsupported InitialRobotState direction field: "
        "%d", dir);
    return Direction::UP;
  }
}
