#ifndef UR_CONTROL_GUI_URCONTROLGUICONFIG_H_
#define UR_CONTROL_GUI_URCONTROLGUICONFIG_H_

//System headers
#include <cstdint>
#include <string>

//Other libraries headers
#include "ur_control_common/layout/config/UrControlCommonLayoutConfig.h"

//Own components headers
#include "ur_control_gui/external_api/config/UrContolGuiExternalBridgeConfig.h"

//Forward declarations

struct UrControlGuiConfig {
  UrControlCommonLayoutConfig commonLayoutCfg;
  UrContolGuiExternalBridgeConfig externalBridgeCfg;
};

#endif /* UR_CONTROL_GUI_URCONTROLGUICONFIG_H_ */

