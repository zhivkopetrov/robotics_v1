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
  PLAYER_ROBOT_IDX = 0,
  ENEMIES_CTN = 3,
  ROBOTS_CTN = 4
};

enum MoveButtonDefines {
  BUTTON_FORWARD,
  BUTTON_ROTATE_LEFT,
  BUTTON_ROTATE_RIGHT,
  MOVE_BUTTONS_CTN
};

enum CoinDefines {
  GOLD_COIN,
  SILVER_COIN,
  BRONZE_COIN,
  COINS_CTN
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

enum class CoinAnimType {
  ROTATE,
  COLLECT,
  RESPAWN
};

enum class Direction {
  UP,
  RIGHT,
  DOWN,
  LEFT
};

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUIDEFINES_H_ */

