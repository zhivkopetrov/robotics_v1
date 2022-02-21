#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUTINTERFACES_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUTINTERFACES_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "robo_collector_common/defines/RoboCollectorFunctionalDefines.h"
#include "robo_common/defines/RoboCommonFunctionalDefines.h"

//Own components headers


//Forward declarations

struct RoboCollectorControllerLayoutInterface {
  EnablePlayerInputCb enablePlayerInputCb;
};

struct RoboCollectorControllerLayoutOutInterface {
  RobotActCb robotActCb;
  HelpActivatedCb helpActivatedCb;
  SettingActivatedCb settingActivatedCb;
};


#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUTINTERFACES_H_ */
