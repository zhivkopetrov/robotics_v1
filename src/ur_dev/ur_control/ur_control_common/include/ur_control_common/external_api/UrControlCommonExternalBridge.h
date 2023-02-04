#ifndef UR_CONTROL_COMMON_URCONTROLCOMMONEXTERNALBRIDGE_H_
#define UR_CONTROL_COMMON_URCONTROLCOMMONEXTERNALBRIDGE_H_

//System headers

//Other libraries headers
#include <rclcpp/node.hpp>
#include <std_msgs/msg/string.hpp>
#include <ur_dashboard_msgs/msg/robot_mode.hpp>
#include <ur_dashboard_msgs/msg/safety_mode.hpp>
#include <urscript_interfaces/srv/ur_script.hpp>
#include "game_engine/defines/ActionEventDefines.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "ur_control_common/defines/UrControlCommonFunctionalDefines.h"
#include "ur_control_common/external_api/config/UrContolCommonExternalBridgeConfig.h"

//Forward declarations

struct UrControlCommonExternalBridgeOutInterface {
  InvokeActionEventCb invokeActionEventCb;
  RobotModeChangeCb robotModeChangeCb;
  SafetyModeChangeCb safetyModeChangeCb;
};

class UrControlCommonExternalBridge: public rclcpp::Node {
public:
  UrControlCommonExternalBridge(const std::string& nodeName);

  ErrorCode init(const UrContolCommonExternalBridgeConfig& cfg,
                 const UrControlCommonExternalBridgeOutInterface &interface);

  void publishURScript(const UrScriptPayload& data) const;

  void invokeURScriptService(const UrScriptPayload& data) const;

private:
  using String = std_msgs::msg::String;
  using UrScript = urscript_interfaces::srv::UrScript;
  using RobotModeType = ur_dashboard_msgs::msg::RobotMode;
  using SafetyModeType = ur_dashboard_msgs::msg::SafetyMode;

  ErrorCode initOutInterface(
      const UrControlCommonExternalBridgeOutInterface &outInterface);
  ErrorCode initCommunication();

  void onRobotModeMsg(const RobotModeType::SharedPtr msg);
  void onSafetyModeMsg(const SafetyModeType::SharedPtr msg);

  UrControlCommonExternalBridgeOutInterface _outInterface;
  rclcpp::Publisher<String>::SharedPtr _urscriptPublisher;
  rclcpp::Client<UrScript>::SharedPtr _urscriptPublisherService;

  rclcpp::Subscription<RobotModeType>::SharedPtr _robotModeSubscriber;
  rclcpp::Subscription<SafetyModeType>::SharedPtr _safetyModeSubscriber;

  //Create different callbacks groups for publishers and subscribers
  //so they can be executed in parallel
  const rclcpp::CallbackGroup::SharedPtr _subscriberCallbackGroup =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  const rclcpp::CallbackGroup::SharedPtr _publishersCallbackGroup =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
};

#endif /* UR_CONTROL_COMMON_URCONTROLCOMMONEXTERNALBRIDGE_H_ */
