#ifndef UR_CONTROL_COMMON_URCONTROLCOMMONLAYOUTINTERFACES_H_
#define UR_CONTROL_COMMON_URCONTROLCOMMONLAYOUTINTERFACES_H_

//System headers
#include <any>

//Other libraries headers

//Own components headers
#include "ur_control_common/defines/UrControlCommonFunctionalDefines.h"

//Forward declarations

struct UrControlCommonLayoutOutInterface {
  PublishURScriptCb publishURScriptCb;
  InvokeDashboardServiceCb invokeDashboardServiceCb;
  std::any buttonHandlerAdditionalOutInterface; //if any
};

struct UrControlCommonLayoutInterface {
  RobotModeChangeCb robotModeChangeCb;
  SafetyModeChangeCb safetyModeChangeCb;
};

#endif /* UR_CONTROL_COMMON_URCONTROLCOMMONLAYOUTINTERFACES_H_ */
