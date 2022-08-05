#ifndef UR_CONTROL_GUI_URCONTROLGUICONFIG_H_
#define UR_CONTROL_GUI_URCONTROLGUICONFIG_H_

//System headers
#include <cstdint>
#include <string>

//Other libraries headers

//Own components headers
#include "ur_control_gui/layout/config/UrControlGuiLayoutConfig.h"

//Forward declarations

struct UrControlGuiConfig {
  UrControlGuiLayoutConfig layoutCfg;
  std::string robotIp;
  uint16_t robotInterfacePort { };
};

#endif /* UR_CONTROL_GUI_URCONTROLGUICONFIG_H_ */

