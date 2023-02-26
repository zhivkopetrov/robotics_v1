#ifndef UR_CONTROL_COMMON_BUTTONHANDLER_H_
#define UR_CONTROL_COMMON_BUTTONHANDLER_H_

//System headers
#include <cstdint>
#include <array>
#include <vector>
#include <string>

//Other libraries headers
#include "utils/ErrorCode.h"

//Own components headers
#include "ur_control_common/layout/entities/button_handler/config/ButtonHandlerConfig.h"
#include "ur_control_common/layout/entities/buttons/UrScriptButton.h"
#include "ur_control_common/layout/entities/buttons/DashboardButton.h"

//Forward declarations
class InputEvent;

struct ButtonHandlerOutInterface {
  PublishURScriptCb publishURScriptCb;
  InvokeDashboardServiceCb invokeDashboardServiceCb;
};

class ButtonHandler {
public:
  ErrorCode init(const ButtonHandlerConfig &cfg,
                 const ButtonHandlerOutInterface &outInterface);
  void draw() const;
  void handleEvent(const InputEvent &e);

private:
  ErrorCode initDashboardButtons(
    const ButtonHandlerConfig &cfg,
    const InvokeDashboardServiceCb &invokeDashboardServiceCb);               

  ErrorCode initGripperButtons(const ButtonHandlerConfig &cfg,
                               const PublishURScriptCb &publishURScriptCb);

  ErrorCode initCommandButtons(const ButtonHandlerConfig &cfg,
                               const PublishURScriptCb &publishURScriptCb);

  ErrorCode loadButtonScripts(const std::string &folderLocation,
                              size_t expectedParsedScriptsCount,
                              std::vector<std::string> &outScripts);

  enum GripperButtonDefines {
    ACTIVATE_GRIPPER_IDX, 
    OPEN_GRIPPER_IDX, 
    CLOSE_GRIPPER_IDX,
    GRIPPER_BUTTONS_COUNT
  };

  enum DashboardButtonDefines {
    POWER_ON_IDX, 
    POWER_OFF_IDX, 
    BRAKE_RELEASE_IDX,
    DASHBOARD_BUTTONS_COUNT
  };

  std::array<DashboardButton, DASHBOARD_BUTTONS_COUNT> _dashboardButtons;
  std::array<UrScriptButton, GRIPPER_BUTTONS_COUNT> _gripperButtons;
  std::vector<UrScriptButton> _commandButtons;
};

#endif /* UR_CONTROL_COMMON_BUTTONHANDLER_H_ */
