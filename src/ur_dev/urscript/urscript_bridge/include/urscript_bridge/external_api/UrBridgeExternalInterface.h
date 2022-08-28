#ifndef URSCRIPT_BRIDGE_URBRIDGEEXTERNALINTERFACE_H
#define URSCRIPT_BRIDGE_URBRIDGEEXTERNALINTERFACE_H

//System headers

//Other libraries headers
#include <rclcpp/node.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ur_msgs/msg/io_states.hpp>
#include <urscript_interfaces/srv/ur_script.hpp>
#include "utils/class/NonCopyable.h"
#include "utils/class/NonMoveable.h"

//Own components headers
#include "urscript_bridge/utils/TcpClient.h"

//Forward declarations

class UrBridgeExternalInterface: public rclcpp::Node,
    public NonCopyable,
    public NonMoveable {
public:
  UrBridgeExternalInterface(const rclcpp::NodeOptions &options);

private:
  using String = std_msgs::msg::String;
  using IOStates = ur_msgs::msg::IOStates;
  using UrScriptSrv = urscript_interfaces::srv::UrScript;
  using Mutex = std::shared_mutex;

  void handleIOState(const IOStates::SharedPtr ioStates);
  void handleUrScript(const String::SharedPtr urScript);
  void handleUrScriptService(
      const std::shared_ptr<UrScriptSrv::Request> request,
      std::shared_ptr<UrScriptSrv::Response> response);

  Mutex mMutex;
  TcpClient mTcpClient;
  uint8_t mRobotPin { };

  IOStates mlatestIoStates;
  rclcpp::Subscription<IOStates>::SharedPtr mIoStatesSubscribtion;
  rclcpp::Subscription<String>::SharedPtr mUrScriptSubscribtion;
  rclcpp::Service<UrScriptSrv>::SharedPtr mUrScriptService;
};

#endif /* URSCRIPT_BRIDGE_URBRIDGEEXTERNALINTERFACE_H */
