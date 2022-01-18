#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUIDEFINES_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUIDEFINES_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers

//Forward declarations

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

