#ifndef UR_CONTROL_GUI_URCONTROLGUIEXTERNALBRIDGE_H_
#define UR_CONTROL_GUI_URCONTROLGUIEXTERNALBRIDGE_H_

//System headers

//Other libraries headers
#include <rclcpp/node.hpp>
#include <std_msgs/msg/string.hpp>
#include <ur_dashboard_msgs/msg/robot_mode.hpp>
#include <ur_dashboard_msgs/msg/safety_mode.hpp>
#include "game_engine/defines/ActionEventDefines.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "ur_control_gui/defines/UrControlGuiFunctionalDefines.h"

//Forward declarations
struct UrContolGuiExternalBridgeConfig;

struct UrControlGuiExternalBridgeOutInterface {
  InvokeActionEventCb invokeActionEventCb;
  RobotModeChangeCb robotModeChangeCb;
  SafetyModeChangeCb safetyModeChangeCb;
};

class UrControlGuiExternalBridge: public rclcpp::Node {
public:
  UrControlGuiExternalBridge();

  ErrorCode init(const UrContolGuiExternalBridgeConfig& cfg,
                 const UrControlGuiExternalBridgeOutInterface &interface);

  void publishURScript(const std::string& data) const;

private:
  using String = std_msgs::msg::String;
  using RobotModeType = ur_dashboard_msgs::msg::RobotMode;
  using SafetyModeType = ur_dashboard_msgs::msg::SafetyMode;

  ErrorCode initOutInterface(
      const UrControlGuiExternalBridgeOutInterface &outInterface);
  ErrorCode initCommunication();

  void onRobotModeMsg(const RobotModeType::SharedPtr msg);
  void onSafetyModeMsg(const SafetyModeType::SharedPtr msg);

  UrControlGuiExternalBridgeOutInterface _outInterface;
  rclcpp::Publisher<String>::SharedPtr _urscriptPublisher;

  rclcpp::Subscription<RobotModeType>::SharedPtr _robotModeSubscriber;
  rclcpp::Subscription<SafetyModeType>::SharedPtr _safetyModeSubscriber;

  std::string _scriptPrefix;
};

#endif /* UR_CONTROL_GUI_URCONTROLGUIEXTERNALBRIDGE_H_ */
