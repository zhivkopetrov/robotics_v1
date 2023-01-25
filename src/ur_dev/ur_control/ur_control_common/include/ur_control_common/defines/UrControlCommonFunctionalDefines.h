#ifndef UR_CONTROL_COMMON_URCONTROLCOMMONFUNCTIONALDEFINES_H_
#define UR_CONTROL_COMMON_URCONTROLCOMMONFUNCTIONALDEFINES_H_

//System headers
#include <functional>
#include <string>

//Other libraries headers

//Own components headers
#include "ur_control_common/defines/UrControlCommonDefines.h"

//Forward declarations

using PublishURScriptCb = std::function<void(const std::string& scriptData)>;
using InvokeDashboardCb = std::function<void(DashboardCommand command)>;
using RobotModeChangeCb = std::function<void(RobotMode mode)>;
using SafetyModeChangeCb = std::function<void(SafetyMode mode)>;

#endif /* UR_CONTROL_COMMON_URCONTROLCOMMONFUNCTIONALDEFINES_H_ */
