#ifndef UR_CONTROL_COMMON_URSCRIPTBUTTONHANDLER_H_
#define UR_CONTROL_COMMON_URSCRIPTBUTTONHANDLER_H_

//System headers
#include <cstdint>
#include <vector>
#include <string>

//Other libraries headers
#include "utils/ErrorCode.h"

//Own components headers
#include "ur_control_common/layout/entities/button_handler/config/UrScriptButtonHandlerConfig.h"
#include "ur_control_common/layout/entities/button_handler/ButtonHandler.h"
#include "ur_control_common/layout/entities/buttons/UrScriptButton.h"

//Forward declarations
class InputEvent;

class UrScriptButtonHandler final : public ButtonHandler {
public:
  ErrorCode init(const ButtonHandlerHighLevelConfig& highLevelCfg,
                 const ButtonHandlerOutInterface& outInterface) override;

  void draw() const override;

  void handleEvent(const InputEvent &e) override;

  ErrorCode setCommandButtonsLockStatus(
    const std::vector<int32_t>& lockBtnIndexes,
    const std::vector<int32_t>& unlockBtnIndexes) override;

  void setGripperButtonsLockStatus(GripperButtonsInputStatus status) override;

private:
  ErrorCode initInternal(const UrScriptButtonHandlerConfig &cfg,
                         const ButtonHandlerOutInterface &outInterface);

  ErrorCode initGripperButtons(const UrScriptButtonHandlerConfig &cfg,
                               const PublishURScriptCb &publishURScriptCb);

  ErrorCode initCommandButtons(const UrScriptButtonHandlerConfig &cfg,
                               const PublishURScriptCb &publishURScriptCb);

  ErrorCode loadButtonScripts(const std::string &folderLocation,
                              size_t expectedParsedScriptsCount,
                              std::vector<std::string> &outScripts);

  std::array<UrScriptButton, GRIPPER_BUTTONS_COUNT> _gripperButtons;
  std::vector<UrScriptButton> _commandButtons;
};

#endif /* UR_CONTROL_COMMON_URSCRIPTBUTTONHANDLER_H_ */
