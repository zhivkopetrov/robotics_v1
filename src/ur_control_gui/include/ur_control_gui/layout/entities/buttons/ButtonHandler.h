#ifndef UR_CONTROL_GUI_BUTTONHANDLER_H_
#define UR_CONTROL_GUI_BUTTONHANDLER_H_

//System headers
#include <cstdint>
#include <array>

//Other libraries headers
#include "utils/ErrorCode.h"

//Own components headers
#include "ur_control_gui/layout/entities/buttons/config/ButtonHandlerConfig.h"
#include "ur_control_gui/layout/entities/buttons/UrScriptButton.h"
#include "ur_control_gui/layout/entities/buttons/DashboardButton.h"

//Forward declarations
class InputEvent;

struct ButtonHandlerOutInterface {
  PublishURScriptCb publishURScriptCb;
  InvokeDashboardCb invokeDashboardCb;
};

class ButtonHandler {
public:
  ErrorCode init(const ButtonHandlerConfig &cfg,
                 const ButtonHandlerOutInterface &outInterface);
  void draw() const;
  void handleEvent(const InputEvent &e);

private:
  ErrorCode initMotionButtons(const ButtonHandlerConfig &cfg,
                              const PublishURScriptCb &publishURScriptCb);

  ErrorCode initGripperButtons(const ButtonHandlerConfig &cfg,
                               const PublishURScriptCb &publishURScriptCb);

  ErrorCode initDashboardButtons(const ButtonHandlerConfig &cfg,
                                 const InvokeDashboardCb &invokeDashboardCb);

  enum MotionButtonDefines {
    GREET_IDX,
    RETURN_HOME_JOINT_IDX,
    WAKE_UP_IDX,
    LEAN_FORWARD_JOINT_IDX,
    RETURN_HOME_LINEAR_IDX,
    LEAN_FORWARD_LINEAR_IDX,
    PICK_AND_PLACE_NON_BLENDED_IDX,
    PICK_AND_PLACE_BLENDED_IDX,

    MOTION_BUTTONS_COUNT
  };

  enum GripperButtonDefines {
    OPEN_GRIPPER_IDX, CLOSE_GRIPPER_IDX,

    GRIPPER_BUTTONS_COUNT
  };

  enum DashboardButtonDefines {
    POWER_ON_IDX, POWER_OFF_IDX, BRAKE_RELEASE_IDX,

    DASHBOARD_BUTTONS_COUNT
  };

  std::array<UrScriptButton, MOTION_BUTTONS_COUNT> _motionButtons;
  std::array<UrScriptButton, GRIPPER_BUTTONS_COUNT> _gripperButtons;
  std::array<DashboardButton, DASHBOARD_BUTTONS_COUNT> _dashboardButtons;
};

#endif /* UR_CONTROL_GUI_BUTTONHANDLER_H_ */
