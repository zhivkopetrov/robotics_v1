#ifndef ROBO_MINER_GUI_ROBOMINERLAYOUTINTERFACES_H_
#define ROBO_MINER_GUI_ROBOMINERLAYOUTINTERFACES_H_

//System headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "robo_common/layout/helpers/RoboCommonLayoutInterfaces.h"

//Own components headers
#include "robo_miner_gui/defines/RoboMinerGuiFunctionalDefines.h"

//Forward declarations
class CollisionWatcher;

struct RoboMinerLayoutInterface {
  RoboCommonLayoutInterface commonLayoutInterface;
  TileReleavedCb tileReleavedCb;
  CrystalMinedCb crystalMinedCb;
};

struct RoboMinerLayoutOutInterface {
  ShutdownGameCb shutdownGameCb;
  TakeScreenshotCb takeScreenshotCb;
  ShutdownControllerCb shutdownControllerCb;
  FieldMapRevelealedCb fieldMapRevelealedCb;
  FinishRobotActCb finishRobotActCb;
  CollisionWatcher *collisionWatcher = nullptr;
};

#endif /* ROBO_MINER_GUI_ROBOMINERLAYOUTINTERFACES_H_ */
