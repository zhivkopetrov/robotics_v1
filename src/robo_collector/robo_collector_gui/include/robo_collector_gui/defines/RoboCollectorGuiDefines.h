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
} //namespace Defines

enum class CoinAnimType {
  ROTATE,
  COLLECT,
  RESPAWN
};
enum class GameType {
  COLLECTOR,
  MINER,
  CLEANER
};

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUIDEFINES_H_ */

