#ifndef ROBO_COLLECTOR_GUI_TURNHELPER_H_
#define ROBO_COLLECTOR_GUI_TURNHELPER_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>

//Other libraries headers

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiFunctionalDefines.h"
#include "robo_collector_gui/entities/robot/RobotAI.h"

//Forward declarations

struct TurnHelperConfig {
  int32_t maxRobots = 0;
  std::vector<RobotActInterface> robotActInterfaces;
  EnablePlayerInputCb enablePlayerInputCb;
  GetFieldDataCb getFieldDataCb;
  char fieldEmptyDataMarker = '!';
  char playerDataMarker = '?';
};

class TurnHelper {
public:
  int32_t init(const TurnHelperConfig& cfg);
  void onRobotFinishAct(int32_t robotId);

private:
  RobotAI _robotAI;

  std::vector<RobotActInterface> _robotActInterfaces;
  EnablePlayerInputCb _enablePlayerInputCb;
  int32_t _activeRobotId = Defines::PLAYER_ROBOT_IDX;
  int32_t _maxRobots;
};

#endif /* ROBO_COLLECTOR_GUI_TURNHELPER_H_ */