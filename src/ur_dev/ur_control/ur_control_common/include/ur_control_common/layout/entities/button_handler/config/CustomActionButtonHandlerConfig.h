#ifndef UR_CONTROL_COMMON_CUSTOMACTIONBUTTONHANDLERCONFIG_H_
#define UR_CONTROL_COMMON_CUSTOMACTIONBUTTONHANDLERCONFIG_H_

//System headers
#include <cstdint>
#include <vector>

//Other libraries headers

//Own components headers
#include "ur_control_common/layout/entities/button_handler/config/ButtonHandlerConfig.h"

//Forward declarations

struct CustomActionButtonHandlerConfig {
  ButtonHandlerConfig baseCfg;
  std::vector<CommandButtonDescription> commandButtonsDescription;
};


#endif /* UR_CONTROL_COMMON_CUSTOMACTIONBUTTONHANDLERCONFIG_H_ */
