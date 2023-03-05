#ifndef UR_CONTROL_COMMON_DASHBOARDBUTTON_H_
#define UR_CONTROL_COMMON_DASHBOARDBUTTON_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "utils/ErrorCode.h"

//Own components headers
#include "ur_control_common/layout/entities/buttons/CommandButton.h"
#include "ur_control_common/defines/UrControlCommonFunctionalDefines.h"

//Forward declarations
class InputEvent;

struct DashboardButtonConfig {
  CommandButtonConfig baseCfg;
  DashboardCommand command = DashboardCommand::POWER_ON_ROBOT;
};

class DashboardButton final : public CommandButton {
public:
  ErrorCode init(const DashboardButtonConfig &cfg,
                 const InvokeDashboardServiceCb &invokeDashboardServiceCb);
  void handleEvent(const InputEvent &e) override;

private:
  InvokeDashboardServiceCb _invokeDashboardServiceCb;
  DashboardCommand _command;
};

#endif /* UR_CONTROL_COMMON_DASHBOARDBUTTON_H_ */
