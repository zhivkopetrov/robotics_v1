#ifndef URSCRIPT_BRIDGE_URSCRIPTCONFIG_H_
#define URSCRIPT_BRIDGE_URSCRIPTCONFIG_H_

//System headers
#include <cstdint>
#include <string>

//Other libraries headers
#include "ros2_game_engine/communicator/config/Ros2CommunicatorConfig.h"

//Own components headers
#include "urscript_bridge/external_api/config/UrBridgeExternalInterfaceConfig.h"

//Forward declarations

struct UrBridgeConfig {
  Ros2CommunicatorConfig ros2CommunicatorCfg;
  UrBridgeExternalInterfaceConfig externalInterfaceConfig;
};

#endif /* URSCRIPT_BRIDGE_URSCRIPTCONFIG_H_ */

