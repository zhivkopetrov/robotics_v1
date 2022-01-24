#ifndef ROBO_COLLECTOR_GUI_TURNHELPER_H_
#define ROBO_COLLECTOR_GUI_TURNHELPER_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiFunctionalDefines.h"

//Forward declarations

struct TurnHelperConfig {
  FinishPlayerActCb finishPlayerActCb;
};

class TurnHelper {
public:
  int32_t init(const TurnHelperConfig& cfg);
  void onRobotFinishAct(int32_t robotId);

private:
  FinishPlayerActCb _finishPlayerActCb;
  int32_t _activeRobotId = Defines::BLINKY_IDX;
};

#endif /* ROBO_COLLECTOR_GUI_TURNHELPER_H_ */
