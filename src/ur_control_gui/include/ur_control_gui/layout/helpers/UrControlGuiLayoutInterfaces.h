#ifndef UR_CONTROL_GUI_URCONTROLGUILAYOUTINTERFACES_H_
#define UR_CONTROL_GUI_URCONTROLGUILAYOUTINTERFACES_H_

//System headers

//Other libraries headers

//Own components headers
#include "ur_control_gui/defines/UrControlGuiFunctionalDefines.h"

//Forward declarations

struct UrControlGuiLayoutOutInterface {
  PublishURScriptCb publishURScriptCb;
  InvokeDashboardCb invokeDashboardCb;
};

struct UrControlGuiLayoutInterface {
  RobotModeChangeCb robotModeChangeCb;
  SafetyModeChangeCb safetyModeChangeCb;
};

#endif /* UR_CONTROL_GUI_URCONTROLGUILAYOUTINTERFACES_H_ */
