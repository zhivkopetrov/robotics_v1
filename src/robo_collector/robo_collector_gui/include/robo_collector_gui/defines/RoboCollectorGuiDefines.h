#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUIDEFINES_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUIDEFINES_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers

//Forward declarations

namespace Defines {
enum RobotDefines {
  ENEMIES_CTN = 3,
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

enum class GameMode {
  UNKNOWN,
  NORMAL,
  EXTENDED,
  SUPER_EXTENDED
};

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

GameMode toGameMode(const int32_t mode);

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUIDEFINES_H_ */

