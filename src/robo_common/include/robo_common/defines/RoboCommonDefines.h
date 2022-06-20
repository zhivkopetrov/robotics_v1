#ifndef ROBO_COMMON_ROBOCOMMONDEFINES_H_
#define ROBO_COMMON_ROBOCOMMONDEFINES_H_

//System headers
#include <cstdint>
#include <array>
#include <vector>
#include <string>

//Other libraries headers

//Own components headers
#include "robo_common/layout/field/FieldPos.h"

//Forward declarations

namespace RoboCommonDefines {
enum RobotDefines {
  PLAYER_ROBOT_IDX = 0
};

enum Markers {
  PLAYER_MARKER = 'B', //for Blinky
  ENEMY_MARKER = 'E',
  SMALL_OBSTACLE_MARKER = 'x',
  BIG_OBSTACLE_MARKER = 'X',
  EMPTY_TILE_MARKER = '.',
  FIELD_OUT_OF_BOUND_MARKER = '#',

  UNKNOWN_FIELD_MARKER = 0
};

enum FieldDefines {
  FIRST_TILE_X_POS = 47, FIRST_TILE_Y_POS = 47, SURROUNDING_TILES_CTN = 3
};
} //namespace RoboCommonDefines

enum class MoveType {
  FORWARD, ROTATE_LEFT, ROTATE_RIGHT,

  UNKNOWN = 0xFF
};

enum class MoveOutcome {
  SUCCESS, COLLISION
};

enum class Direction {
  UP, RIGHT, DOWN, LEFT
};

enum class RotationDir {
  LEFT, RIGHT
};

enum class Achievement {
  SINGLE_STAR, DOUBLE_STAR, TRIPLE_STAR
};

struct RobotState {
  FieldPos fieldPos;
  int32_t robotId = 0;
  Direction dir = Direction::UP;
};

enum class RobotFieldMarkers {
  ENABLED, DISABLED
};

enum class ObstacleHandlerApproachOverlayStatus {
  ENABLED, DISABLED
};

enum class EndGameOutcome {
  WIN, LOSE
};

using SurroundingTiles = std::array<uint8_t, RoboCommonDefines::SURROUNDING_TILES_CTN>;
using FieldData = std::vector<std::vector<char>>;

struct FieldDescription {
  FieldData data;
  int32_t tileWidth = 0;
  int32_t tileHeight = 0;
  int32_t rows = 0;
  int32_t cols = 0;
  int32_t emptyTilesCount = 0;
  int32_t obstacleTilesCount = 0;
  char emptyDataMarker = RoboCommonDefines::EMPTY_TILE_MARKER;
  std::vector<char> obstacleMarkers { RoboCommonDefines::BIG_OBSTACLE_MARKER,
      RoboCommonDefines::SMALL_OBSTACLE_MARKER };
};

struct UserData {
  std::string user = "not_set";
  std::string repository = "not_set";
  std::string commitSha = "not_set";
};

inline constexpr int32_t INDICATOR_PANEL_MAX_VALUE = 400;

#endif /* ROBO_COMMON_ROBOCOMMONDEFINES_H_ */
