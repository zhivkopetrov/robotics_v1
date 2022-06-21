#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUTINTERFACES_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUTINTERFACES_H_

//System headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "robo_common/layout/helpers/RoboCommonLayoutInterfaces.h"
#include "robo_collector_common/defines/RoboCollectorFunctionalDefines.h"

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiFunctionalDefines.h"

//Forward declarations
class CollisionWatcher;

struct RoboCollectorLayoutInterface {
  RoboCommonLayoutInterface commonLayoutInterface;
  EnablePlayerInputCb enablePlayerInputCb;
  MoveButtonClickCb moveButtonClickCb;
  std::vector<RobotActInterface> enemyRobotActInterfaces;
};

struct RoboCollectorLayoutOutInterface {
  ShutdownControllerCb shutdownControllerCb;
  TakeScreenshotCb takeScreenshotCb;
  ShutdownGameCb shutdownGameCb;
  FinishRobotActCb finishRobotActCb;
  IsPlayerTurnActiveCb isPlayerTurnActiveCb;
  CollisionWatcher *collisionWatcher = nullptr;
};


#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUTINTERFACES_H_ */
