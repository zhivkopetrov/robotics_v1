#ifndef UR_CONTROL_GUI_DASHBOARDBUTTON_H_
#define UR_CONTROL_GUI_DASHBOARDBUTTON_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "utils/ErrorCode.h"

//Own components headers
#include "ur_control_gui/layout/entities/buttons/config/ButtonHandlerConfig.h"
#include "ur_control_gui/layout/entities/buttons/CommandButton.h"
#include "ur_control_gui/defines/UrControlGuiFunctionalDefines.h"

//Forward declarations
class InputEvent;

struct DashboardButtonConfig {
  CommandButtonConfig baseCfg;
  DashboardCommand command = DashboardCommand::POWER_ON_ROBOT;
};

class DashboardButton final : public CommandButton {
public:
  ErrorCode init(const DashboardButtonConfig &cfg,
                 const InvokeDashboardCb &invokeDashboardCb);
  void handleEvent(const InputEvent &e) override;

private:
  InvokeDashboardCb _invokeDashboardCb;
  DashboardCommand _command;
};

#endif /* UR_CONTROL_GUI_DASHBOARDBUTTON_H_ */
