#ifndef ROBO_COMMON_ROBOCOMMONDEFINES_H_
#define ROBO_COMMON_ROBOCOMMONDEFINES_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>

//Other libraries headers

//Own components headers

//Forward declarations

using FieldData = std::vector<std::vector<char>>;

namespace RoboCommonDefines {
enum RobotDefines {
  PLAYER_ROBOT_IDX = 0
};

enum FieldDefines {
  FIELD_ROWS = 6,
  FIELD_COLS = 7,
  FIRST_TILE_X_POS = 47,
  FIRST_TILE_Y_POS = 47,
  TILE_WIDTH = 160,
  TILE_HEIGHT = 160
};
} //namespace Defines

enum class MoveType {
  UNKNOWN,
  FORWARD,
  ROTATE_LEFT,
  ROTATE_RIGHT
};

enum class Direction {
  UP,
  RIGHT,
  DOWN,
  LEFT
};

enum class RotationDir {
  LEFT, RIGHT
};

#endif /* ROBO_COMMON_ROBOCOMMONDEFINES_H_ */
