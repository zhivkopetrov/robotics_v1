#ifndef URSCRIPT_BRINDGE_URBRIDGEEXTERNALINTERFACECONFIG_H_
#define URSCRIPT_BRINDGE_URBRIDGEEXTERNALINTERFACECONFIG_H_

//System headers
#include <cstdint>
#include <string>

//Other libraries headers

//Own components headers

//Forward declarations

struct UrBridgeExternalInterfaceConfig {
  std::string robotIp;
  uint16_t robotInterfacePort { };
  uint32_t urScriptServiceReadyPin { };
  bool verboseLogging = false;
};

#endif /* URSCRIPT_BRINDGE_URBRIDGEEXTERNALINTERFACECONFIG_H_ */
