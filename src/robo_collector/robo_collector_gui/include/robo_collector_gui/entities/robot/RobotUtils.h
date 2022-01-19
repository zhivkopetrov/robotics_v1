#ifndef ROBO_COLLECTOR_GUI_ENTITIES_ROBOTUTILS_H_
#define ROBO_COLLECTOR_GUI_ENTITIES_ROBOTUTILS_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiDefines.h"

//Forward declarations

class RobotUtils {
public:
  RobotUtils() = delete;

  static Direction getDirAfterRotation(Direction currDir, bool isLeftRotation);

  static double getRotationDegFromDir(Direction dir);
};

#endif /* ROBO_COLLECTOR_GUI_ENTITIES_ROBOTUTILS_H_ */
