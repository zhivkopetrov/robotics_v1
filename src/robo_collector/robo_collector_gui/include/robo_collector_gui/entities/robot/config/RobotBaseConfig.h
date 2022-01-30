#ifndef ROBO_COLLECTOR_GUI_ROBOTBASECONFIG_H_
#define ROBO_COLLECTOR_GUI_ROBOTBASECONFIG_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers

//Forward declarations

struct RobotBaseConfig {
  uint64_t playerRsrcId = 0;
  uint64_t enemiesRsrcId = 0;
  int32_t moveAnimStartTimerId = 0;
  int32_t wallCollisionAnimStartTimerId = 0;
  int32_t robotCollisionAnimStartTimerId = 0;
};

#endif /* ROBO_COLLECTOR_GUI_ROBOTBASECONFIG_H_ */
