#ifndef URSCRIPT_BRIDGE_URSCRIPTROS2PARAMPROVIDER_H_
#define URSCRIPT_BRIDGE_URSCRIPTROS2PARAMPROVIDER_H_

//System headers
#include <string>

//Other libraries headers
#include <rclcpp/node.hpp>
#include "ros2_game_engine/communicator/config/Ros2CommunicatorConfig.h"

//Own components headers

//Forward declarations

struct UrBridgeRos2Params {
  Ros2CommunicatorConfig ros2CommunicatorCfg;
  std::string robotIp;
  uint16_t robotInterfacePort { };
  uint32_t urScriptServiceReadyPin { };
  bool verboseLogging = false;

  void print() const;
  void validate();
};

class UrBridgeRos2ParamProvider: public rclcpp::Node {
public:
  UrBridgeRos2ParamProvider();

  UrBridgeRos2Params getParams();

private:
  UrBridgeRos2Params _params;
};

#endif /* URSCRIPT_BRIDGE_URSCRIPTROS2PARAMPROVIDER_H_ */
