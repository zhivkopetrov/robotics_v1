#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUTINTERFACES_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUTINTERFACES_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>

//Other libraries headers

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiFunctionalDefines.h"
#include "robo_collector_gui/layout/entities/robot/RobotAI.h"

//Forward declarations
class CollisionWatcher;

struct RoboCollectorLayoutInterface {
  EnablePlayerInputCb enablePlayerInputCb;
  GetFieldDataCb getFieldDataCb;
  MoveButtonClickCb moveButtonClickCb;
  std::vector<RobotActInterface> robotActInterfaces;
};

struct RoboCollectorLayoutOutInterface {
  IsPlayerTurnActiveCb isPlayerTurnActiveCb;
  FinishRobotActCb finishRobotActCb;
  CollisionWatcher *collisionWatcher = nullptr;
};


#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUTINTERFACES_H_ */
