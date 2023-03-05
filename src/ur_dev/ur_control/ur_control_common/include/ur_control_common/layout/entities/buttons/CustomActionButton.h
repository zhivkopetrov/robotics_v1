#ifndef UR_CONTROL_COMMON_CUSTOMACTIONBUTTON_H_
#define UR_CONTROL_COMMON_CUSTOMACTIONBUTTON_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "utils/ErrorCode.h"

//Own components headers
#include "ur_control_common/layout/entities/buttons/CommandButton.h"
#include "ur_control_common/defines/UrControlCommonFunctionalDefines.h"

//Forward declarations
class InputEvent;

class CustomActionButton final : public CommandButton {
public:
  ErrorCode init(const CommandButtonConfig &cfg,
                 const CustomActionCb &customActionCb);
  void handleEvent(const InputEvent &e) override;

private:
  CustomActionCb _customActionCb;
};

#endif /* UR_CONTROL_COMMON_CUSTOMACTIONBUTTON_H_ */
