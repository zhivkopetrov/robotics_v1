#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUIDEFINES_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUIDEFINES_H_

//System headers
#include <cstdint>

//Other libraries headers

//Own components headers

//Forward declarations

namespace Defines {
enum RobotDefines {
  ENEMIES_CTN = 3,
  ROBOTS_CTN = 4
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

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUIDEFINES_H_ */

