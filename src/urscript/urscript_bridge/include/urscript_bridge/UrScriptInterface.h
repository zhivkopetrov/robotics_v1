#ifndef URSCRIPT_INTERFACE_URSCRIPTINTERFACE_H
#define URSCRIPT_INTERFACE_URSCRIPTINTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ur_msgs/msg/io_states.hpp>
#include <urscript_interfaces/srv/ur_script.hpp>

#include "urscript_bridge/TcpClient.h"

class UrScriptInterface: public rclcpp::Node {
public:
  explicit UrScriptInterface(const rclcpp::NodeOptions &options);

  // Disable copy and move
  UrScriptInterface(const UrScriptInterface &other) = delete;
  UrScriptInterface(UrScriptInterface &&other) noexcept = delete;

  ~UrScriptInterface() noexcept = default;

  // Disable copy and move
  UrScriptInterface& operator=(const UrScriptInterface &other) = delete;
  UrScriptInterface& operator=(UrScriptInterface &&other) noexcept = delete;

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

  uint8_t mRobotPin;
  Mutex mMutex;
  ur_msgs::msg::IOStates::SharedPtr mIoStates;
  rclcpp::Subscription<IOStates>::SharedPtr mIoStatesSubscribtion;
  rclcpp::Subscription<String>::SharedPtr mUrScriptSubscribtion;
  rclcpp::Service<UrScriptSrv>::SharedPtr mUrScriptService;
  TcpClient mTcpClient;
  rclcpp::Logger mLogger;
};

#endif /* URSCRIPT_INTERFACE_URSCRIPTINTERFACE_H */
