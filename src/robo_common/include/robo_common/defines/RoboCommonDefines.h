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

struct FieldDescription {
  FieldData data;
  int32_t tileWidth = 0;
  int32_t tileHeight = 0;
  int32_t rows = 0;
  int32_t cols = 0;
  char emptyDataMarker = '.';
  char hardObstacleMarker = '#';
};

namespace RoboCommonDefines {
enum RobotDefines {
  PLAYER_ROBOT_IDX = 0
};

enum FieldDefines {
  FIRST_TILE_X_POS = 47,
  FIRST_TILE_Y_POS = 47,
  HARD_OBSTACLE_MARKER = 'X'
};
} //namespace RoboCommonDefines

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

enum class Achievement {
  SINGLE_STAR,
  DOUBLE_STAR,
  TRIPLE_STAR
};

#endif /* ROBO_COMMON_ROBOCOMMONDEFINES_H_ */
