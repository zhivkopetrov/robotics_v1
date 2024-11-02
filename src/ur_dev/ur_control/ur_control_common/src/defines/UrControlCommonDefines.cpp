//Corresponding header
#include "ur_control_common/defines/UrControlCommonDefines.h"

//System headers

//Other libraries headers
#include "utils/data_type/EnumClassUtils.h"
#include "utils/log/Log.h"

//Own components headers

std::string toString(MotionAction action) {
  switch (action)
  {
  case MotionAction::START:
    return "MotionAction::START";

  case MotionAction::GRACEFUL_STOP:
    return "MotionAction::GRACEFUL_STOP";

  case MotionAction::ABORT:
    return "MotionAction::ABORT";

  case MotionAction::RECOVER:
    return "MotionAction::RECOVER";
  
  default:
    LOGERR(
      "Error, received unsupported MotionAction: [%d]", getEnumValue(action));
    return "MotionAction::Unsupported";
  }
}