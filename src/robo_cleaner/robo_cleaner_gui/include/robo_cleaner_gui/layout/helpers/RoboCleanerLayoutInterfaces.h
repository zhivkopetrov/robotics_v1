#ifndef ROBO_CLEANER_GUI_ROBOCLEANERLAYOUTINTERFACES_H_
#define ROBO_CLEANER_GUI_ROBOCLEANERLAYOUTINTERFACES_H_

//System headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "robo_common/layout/helpers/RoboCommonLayoutInterfaces.h"

//Own components headers
#include "robo_cleaner_gui/defines/RoboCleanerGuiFunctionalDefines.h"

//Forward declarations
class CollisionWatcher;

struct RoboCleanerLayoutInterface {
  RoboCommonLayoutInterface commonLayoutInterface;
  ModifyRubbishWidgetCb modifyRubbishWidgetCb;
  ModifyEnergyLevelCb modifyEnergyLevelCb;
  TileReleavedCb tileReleavedCb;
  TileCleanedCb tileCleanedCb;
};

struct RoboCleanerLayoutOutInterface {
  ShutdownGameCb shutdownGameCb;
  FinishRobotActCb finishRobotActCb;
  PlayerRobotDamageCollisionCb playerRobotDamageCollisionCb;
  FieldMapRevelealedCb fieldMapRevelealedCb;
  FieldMapCleanedCb fieldMapCleanedCb;
  ObjechApproachOverlayTriggeredCb objectApproachOverlayTriggeredCb;
  CollisionWatcher *collisionWatcher = nullptr;
};

#endif /* ROBO_CLEANER_GUI_ROBOCLEANERLAYOUTINTERFACES_H_ */
