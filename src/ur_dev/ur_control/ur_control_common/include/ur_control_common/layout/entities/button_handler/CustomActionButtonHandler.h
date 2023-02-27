#ifndef UR_CONTROL_COMMON_CUSTOMACTIONBUTTONHANDLER_H_
#define UR_CONTROL_COMMON_CUSTOMACTIONBUTTONHANDLER_H_

//System headers
#include <cstdint>
#include <vector>
#include <string>

//Other libraries headers
#include "utils/ErrorCode.h"

//Own components headers
#include "ur_control_common/defines/UrControlCommonFunctionalDefines.h"
#include "ur_control_common/layout/entities/button_handler/config/CustomActionButtonHandlerConfig.h"
#include "ur_control_common/layout/entities/button_handler/ButtonHandler.h"
#include "ur_control_common/layout/entities/buttons/UrScriptButton.h"
#include "ur_control_common/layout/entities/buttons/CustomActionButton.h"

//Forward declarations
class InputEvent;

class CustomActionButtonHandler final : public ButtonHandler {
public:
  ErrorCode init(const ButtonHandlerHighLevelConfig& highLevelCfg,
                 const ButtonHandlerOutInterface& outInterface) override;

  void draw() const override;

  void handleEvent(const InputEvent &e) override;

private:
  ErrorCode initInternal(
    const CustomActionButtonHandlerConfig &cfg,
    const CustomActionButtonHandlerOutInterface &outInterface);

  ErrorCode initGripperButtons(const CustomActionButtonHandlerConfig &cfg,
                               const CustomActionButtonCbs &buttonCbs);

  ErrorCode initCommandButtons(const CustomActionButtonHandlerConfig &cfg,
                               const CustomActionButtonCbs &buttonCbs);

  std::array<CustomActionButton, GRIPPER_BUTTONS_COUNT> _gripperButtons;
  std::vector<CustomActionButton> _commandButtons;
};

#endif /* UR_CONTROL_COMMON_CUSTOMACTIONBUTTONHANDLER_H_ */
