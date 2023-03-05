#ifndef UR_CONTROL_COMMON_URSCRIPTBUTTON_H_
#define UR_CONTROL_COMMON_URSCRIPTBUTTON_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "utils/ErrorCode.h"

//Own components headers
#include "ur_control_common/layout/entities/buttons/CommandButton.h"
#include "ur_control_common/defines/UrControlCommonFunctionalDefines.h"

//Forward declarations
class InputEvent;

struct UrScriptButtonConfig {
  CommandButtonConfig baseCfg;
  std::string commandData;
};

class UrScriptButton final : public CommandButton {
public:
  ErrorCode init(const UrScriptButtonConfig &cfg,
                 const PublishURScriptCb &publishURScriptCb);
  void handleEvent(const InputEvent &e) override;

private:
  PublishURScriptCb _publishURScriptCb;
  std::string _commandData;
};

#endif /* UR_CONTROL_COMMON_URSCRIPTBUTTON_H_ */
