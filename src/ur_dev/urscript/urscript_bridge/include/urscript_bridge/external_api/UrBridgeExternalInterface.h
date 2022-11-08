#ifndef URSCRIPT_BRIDGE_URBRIDGEEXTERNALINTERFACE_H
#define URSCRIPT_BRIDGE_URBRIDGEEXTERNALINTERFACE_H

//System headers

//Other libraries headers
#include <rclcpp/node.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ur_msgs/msg/io_states.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "urscript_interfaces/srv/ur_script.hpp"
#include "urscript_interfaces/srv/get_eef_angle_axis.hpp"
#include "utils/class/NonCopyable.h"
#include "utils/class/NonMoveable.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "urscript_bridge/external_api/config/UrBridgeExternalInterfaceConfig.h"
#include "urscript_bridge/utils/TcpClient.h"

//Forward declarations

class UrBridgeExternalInterface: public rclcpp::Node,
    public NonCopyable,
    public NonMoveable {
public:
  UrBridgeExternalInterface();

  ErrorCode init(const UrBridgeExternalInterfaceConfig& cfg);

private:
  using String = std_msgs::msg::String;
  using IOStates = ur_msgs::msg::IOStates;
  using UrScriptSrv = urscript_interfaces::srv::UrScript;
  using GetEefAngleAxis = urscript_interfaces::srv::GetEefAngleAxis;
  using Mutex = std::shared_mutex;

  enum class PinState {
    UNTOGGLED, TOGGLED
  };

  void initTogglePinMessagesPayload(uint32_t pin);
  ErrorCode initCommunication();

  void handleIOState(const IOStates::SharedPtr ioStates);

  void handleUrScript(const String::SharedPtr urScript);

  void handleUrScriptService(
      const std::shared_ptr<UrScriptSrv::Request> request,
      std::shared_ptr<UrScriptSrv::Response> response);

  void handleGetEefAngleAxisService(
      const std::shared_ptr<GetEefAngleAxis::Request> request,
      std::shared_ptr<GetEefAngleAxis::Response> response);

  void waitForPinState(PinState state);

  TcpClient mTcpClient;
  uint32_t mUrScriptServiceReadyPin { };

  std::string mTogglePinMsgPayload;
  std::string mUntogglePinMsgPayload;

  Mutex mIoMutex;
  IOStates mLatestIoStates;

  std::unique_ptr<tf2_ros::Buffer> mTfBuffer;
  std::unique_ptr<tf2_ros::TransformListener> mTfListener;
  Mutex mTfMutex;

  rclcpp::Subscription<IOStates>::SharedPtr mIoStatesSubscribtion;
  rclcpp::Subscription<String>::SharedPtr mUrScriptSubscribtion;
  rclcpp::Service<UrScriptSrv>::SharedPtr mUrScriptService;
  rclcpp::Service<GetEefAngleAxis>::SharedPtr mGetEefAngleAxisService;

  const rclcpp::CallbackGroup::SharedPtr mCallbackGroup = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
};

#endif /* URSCRIPT_BRIDGE_URBRIDGEEXTERNALINTERFACE_H */
