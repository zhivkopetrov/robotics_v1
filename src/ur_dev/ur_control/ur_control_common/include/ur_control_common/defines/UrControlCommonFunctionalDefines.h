#ifndef UR_CONTROL_COMMON_URCONTROLCOMMONFUNCTIONALDEFINES_H_
#define UR_CONTROL_COMMON_URCONTROLCOMMONFUNCTIONALDEFINES_H_

//System headers
#include <functional>
#include <string>

//Other libraries headers

//Own components headers
#include "ur_control_common/defines/UrControlCommonDefines.h"
#include "urscript_common/defines/UrScriptDefines.h"

//Forward declarations

using PublishURScriptCb = std::function<void(const UrScriptPayload& data)>;
using InvokeURScriptServiceCb = 
  std::function<void(const UrScriptPayload& data)>;
using InvokeURScriptPreemptServiceCb = std::function<void()>;
using InvokeDashboardServiceCb = std::function<void(DashboardCommand command)>;
using RobotModeChangeCb = std::function<void(RobotMode mode)>;
using SafetyModeChangeCb = std::function<void(SafetyMode mode)>;

using UrscriptsBatchDoneCb = std::function<void()>;
using UrscriptDoneCb = std::function<void()>;

struct UrscriptCommand {
  UrScriptPayload data;
  UrscriptDoneCb doneCb = nullptr; //optional
  MotionExecutionPolicy policy = MotionExecutionPolicy::BLOCKING;
};

using DispatchUscriptsAsyncCb = 
  std::function<void(const std::vector<UrscriptCommand>& commands, 
                     const UrscriptsBatchDoneCb& batchDoneCb)>;

#endif /* UR_CONTROL_COMMON_URCONTROLCOMMONFUNCTIONALDEFINES_H_ */
