#ifndef ROBO_COMMON_ROBOCOMMONLAYOUTINTERFACES_H_
#define ROBO_COMMON_ROBOCOMMONLAYOUTINTERFACES_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"
#include "robo_common/layout/entities/robot/helpers/RobotActInterface.h"

//Forward declarations
class CollisionWatcher;

struct RoboCommonLayoutInterface {
  SetFieldDataMarkerCb setFieldDataMarkerCb;
  ResetFieldDataMarkerCb resetFieldDataMarkerCb;
  GetFieldDescriptionCb getFieldDescriptionCb;
  RobotActInterface playerRobotActInterface;
  StartGameLostAnimCb startGameLostAnimCb;
  StartGameWonAnimCb startGameWonAnimCb;
  StartAchievementWonAnimCb startAchievementWonAnimCb;
};

struct RoboCommonLayoutOutInterface {
  ShutdownGameCb shutdownGameCb;
  FinishRobotActCb finishRobotActCb;
  PlayerDamageCb playerDamageCb;
  CollisionWatcher *collisionWatcher = nullptr;
};

#endif /* ROBO_COMMON_ROBOCOMMONLAYOUTINTERFACES_H_ */
