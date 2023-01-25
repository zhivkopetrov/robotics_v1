#ifndef UR_CONTROL_COMMON_URCONTROLCOMMONLAYOUTINTERFACES_H_
#define UR_CONTROL_COMMON_URCONTROLCOMMONLAYOUTINTERFACES_H_

//System headers

//Other libraries headers

//Own components headers
#include "ur_control_common/defines/UrControlCommonFunctionalDefines.h"

//Forward declarations

struct UrControlCommonLayoutOutInterface {
  PublishURScriptCb publishURScriptCb;
  InvokeDashboardCb invokeDashboardCb;
};

struct UrControlCommonLayoutInterface {
  RobotModeChangeCb robotModeChangeCb;
  SafetyModeChangeCb safetyModeChangeCb;
};

#endif /* UR_CONTROL_COMMON_URCONTROLCOMMONLAYOUTINTERFACES_H_ */
