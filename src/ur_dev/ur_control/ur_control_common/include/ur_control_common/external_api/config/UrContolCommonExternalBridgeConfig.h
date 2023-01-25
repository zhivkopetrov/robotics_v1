#ifndef UR_CONTROL_COMMON_URCONTOLCOMMONEXTERNALBRIDGECONFIG_H_
#define UR_CONTROL_COMMON_URCONTOLCOMMONEXTERNALBRIDGECONFIG_H_

//System headers
#include <cstdint>
#include <string>

//Other libraries headers

//Own components headers

//Forward declarations

struct UrContolCommonExternalBridgeConfig {
  std::string robotIp;
  uint16_t robotInterfacePort { };
};

#endif /* UR_CONTROL_COMMON_URCONTOLCOMMONEXTERNALBRIDGECONFIG_H_ */
