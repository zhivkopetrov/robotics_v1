#ifndef ROBO_COLLECTOR_GUI_TURNHELPER_H_
#define ROBO_COLLECTOR_GUI_TURNHELPER_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "robo_common/layout/entities/robot/helpers/RobotActInterface.h"
#include "robo_collector_common/defines/RoboCollectorFunctionalDefines.h"

//Own components headers
#include "robo_collector_gui/layout/entities/robot/RobotAI.h"

//Forward declarations

struct TurnHelperConfig {
  int32_t maxRobots = 0;
  std::vector<RobotActInterface> robotActInterfaces;
  EnablePlayerInputCb enablePlayerInputCb;
  GetFieldDescriptionCb getFieldDescriptionCb;
  char fieldEnemyMarker = '!';
};

class TurnHelper {
public:
  int32_t init(const TurnHelperConfig& cfg);
  void onRobotFinishAct(int32_t robotId);
  bool isPlayerTurnActive() const;

private:
  RobotAI _robotAI;

  std::vector<RobotActInterface> _robotActInterfaces;
  EnablePlayerInputCb _enablePlayerInputCb;
  int32_t _activeRobotId = RoboCommonDefines::PLAYER_ROBOT_IDX;
  int32_t _maxRobots = 0;
};

#endif /* ROBO_COLLECTOR_GUI_TURNHELPER_H_ */
