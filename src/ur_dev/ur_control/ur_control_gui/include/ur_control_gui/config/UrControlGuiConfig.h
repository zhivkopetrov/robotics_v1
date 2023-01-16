#ifndef UR_CONTROL_GUI_URCONTROLGUICONFIG_H_
#define UR_CONTROL_GUI_URCONTROLGUICONFIG_H_

//System headers
#include <cstdint>
#include <string>

//Other libraries headers

//Own components headers
#include "ur_control_gui/layout/config/UrControlGuiLayoutConfig.h"
#include "ur_control_gui/external_api/config/UrContolGuiExternalBridgeConfig.h"

//Forward declarations

struct UrControlGuiConfig {
  UrControlGuiLayoutConfig layoutCfg;
  UrContolGuiExternalBridgeConfig urContolGuiExternalBridgeCfg;
};

#endif /* UR_CONTROL_GUI_URCONTROLGUICONFIG_H_ */

