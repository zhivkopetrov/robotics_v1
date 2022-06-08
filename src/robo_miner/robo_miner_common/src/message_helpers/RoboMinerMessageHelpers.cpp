//Corresponding header
#include "robo_miner_common/message_helpers/RoboMinerMessageHelpers.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "robo_miner_interfaces/msg/robot_position_response.hpp"
#include "robo_miner_interfaces/srv/robot_move.hpp"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/Log.h"

//Own components headers

using robo_miner_interfaces::srv::RobotMove;
using robo_miner_interfaces::msg::RobotMoveType;
using robo_miner_interfaces::msg::RobotPositionResponse;

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
    LOGERR("Error, received unsupported RobotMoveType field: %hhu", moveType);
    return MoveType::UNKNOWN;
  }
}

int32_t getRobotDirectionField(Direction dir) {
  switch (dir) {
  case Direction::UP:
    return RobotPositionResponse::DIRECTION_UP;
  case Direction::RIGHT:
    return RobotPositionResponse::DIRECTION_RIGHT;
  case Direction::DOWN:
    return RobotPositionResponse::DIRECTION_DOWN;
  case Direction::LEFT:
    return RobotPositionResponse::DIRECTION_LEFT;
  default:
    LOGERR("Error, received unsupported Direction: %d", dir);
    return RobotPositionResponse::DIRECTION_UP;
  }
}
Direction getRobotDirection(int32_t dir) {
  switch (dir) {
  case RobotPositionResponse::DIRECTION_UP:
    return Direction::UP;
  case RobotPositionResponse::DIRECTION_RIGHT:
    return Direction::RIGHT;
  case RobotPositionResponse::DIRECTION_DOWN:
    return Direction::DOWN;
  case RobotPositionResponse::DIRECTION_LEFT:
    return Direction::LEFT;
  default:
    LOGERR("Error, received unsupported RobotPositionResponse direction field: "
        "%d", dir);
    return Direction::UP;
  }
}
