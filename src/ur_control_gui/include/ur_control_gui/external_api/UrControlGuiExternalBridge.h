#ifndef UR_CONTROL_GUI_URCONTROLGUIEXTERNALBRIDGE_H_
#define UR_CONTROL_GUI_URCONTROLGUIEXTERNALBRIDGE_H_

//System headers

//Other libraries headers
#include <rclcpp/node.hpp>
#include <std_msgs/msg/string.hpp>
#include "game_engine/defines/ActionEventDefines.h"
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations

struct UrControlGuiExternalBridgeOutInterface {
  InvokeActionEventCb invokeActionEventCb;
  SystemShutdownCb systemShutdownCb;
};

class UrControlGuiExternalBridge: public rclcpp::Node {
public:
  UrControlGuiExternalBridge();

  ErrorCode init(const UrControlGuiExternalBridgeOutInterface &interface);

  void publishURScript(const std::string& data) const;

private:
  using String = std_msgs::msg::String;

  ErrorCode initOutInterface(
      const UrControlGuiExternalBridgeOutInterface &outInterface);
  ErrorCode initCommunication();

  UrControlGuiExternalBridgeOutInterface _outInterface;
  rclcpp::Publisher<String>::SharedPtr _urscriptPublisher;
};

#endif /* UR_CONTROL_GUI_URCONTROLGUIEXTERNALBRIDGE_H_ */
