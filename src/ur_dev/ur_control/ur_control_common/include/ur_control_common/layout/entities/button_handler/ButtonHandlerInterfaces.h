#ifndef UR_CONTROL_COMMON_BUTTONHANDLERINTERFACES_H_
#define UR_CONTROL_COMMON_BUTTONHANDLERINTERFACES_H_

//System headers
#include <any>
#include <vector>

//Other libraries headers

//Own components headers
#include "ur_control_common/defines/UrControlCommonFunctionalDefines.h"

//Forward declarations

using CustomActionButtonCbs = std::vector<CustomActionCb>;

struct ButtonHandlerOutInterface {
  PublishURScriptCb publishURScriptCb;
  InvokeDashboardServiceCb invokeDashboardServiceCb;
  std::any additionalOutInterface; //if any
};

struct CustomActionButtonHandlerCbs {
  CustomActionButtonCbs commandButtonCbs;
  CustomActionButtonCbs gripperButtonCbs;
};

struct CustomActionButtonHandlerOutInterface {
  PublishURScriptCb publishURScriptCb;
  InvokeDashboardServiceCb invokeDashboardServiceCb;
  CustomActionButtonHandlerCbs customActionButtonHandlerCbs;
};

#endif /* UR_CONTROL_COMMON_BUTTONHANDLERINTERFACES_H_ */
