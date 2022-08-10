#ifndef UR_CONTROL_GUI_URCONTOLGUIEXTERNALBRIDGECONFIG_H_
#define UR_CONTROL_GUI_URCONTOLGUIEXTERNALBRIDGECONFIG_H_

//System headers
#include <cstdint>
#include <string>

//Other libraries headers

//Own components headers

//Forward declarations

struct UrContolGuiExternalBridgeConfig {
  std::string robotIp;
  uint16_t robotInterfacePort { };
};

#endif /* UR_CONTROL_GUI_URCONTOLGUIEXTERNALBRIDGECONFIG_H_ */
