#ifndef UR_CONTROL_GUI_URCONTROLGUIFUNCTIONALDEFINES_H_
#define UR_CONTROL_GUI_URCONTROLGUIFUNCTIONALDEFINES_H_

//System headers
#include <functional>
#include <string>

//Other libraries headers

//Own components headers
#include "ur_control_gui/defines/UrControlGuiDefines.h"

//Forward declarations

using PublishURScriptCb = std::function<void(const std::string& scriptData)>;
using InvokeDashboardCb = std::function<void(DashboardCommand command)>;
using RobotModeChangeCb = std::function<void(RobotMode mode)>;
using SafetyModeChangeCb = std::function<void(SafetyMode mode)>;

#endif /* UR_CONTROL_GUI_URCONTROLGUIFUNCTIONALDEFINES_H_ */
