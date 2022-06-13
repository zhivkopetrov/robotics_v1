#ifndef ROBO_COLLECTOR_GUI_COLLECTORCONTROLLEREXTERNALBRIDGE_H_
#define ROBO_COLLECTOR_GUI_COLLECTORCONTROLLEREXTERNALBRIDGE_H_

//System headers

//Other libraries headers
#include <rclcpp/node.hpp>
#include <std_msgs/msg/empty.hpp>
#include "robo_collector_interfaces/msg/robot_move_type.hpp"
#include "robo_collector_common/defines/RoboCollectorFunctionalDefines.h"
#include "game_engine/defines/ActionEventDefines.h"
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations

struct CollectorControllerExternalBridgeOutInterface {
  InvokeActionEventCb invokeActionEventCb;
  MoveButtonClickCb moveButtonClickCb;
  ToggleHelpPageCb toggleHelpPageCb;
  ToggleDebugInfoCb toggleDebugInfoCb;
};

class CollectorControllerExternalBridge: public rclcpp::Node {
public:
  CollectorControllerExternalBridge();

  ErrorCode init(
      const CollectorControllerExternalBridgeOutInterface &interface);

  void publishEnablePlayerInput();

  void publishShutdownController();

private:
  using Empty = std_msgs::msg::Empty;
  using RobotMoveType = robo_collector_interfaces::msg::RobotMoveType;

  enum class ControllerStatus {
    IDLE, ACTIVE, SHUTTING_DOWN
  };

  ErrorCode initOutInterface(
      const CollectorControllerExternalBridgeOutInterface &outInterface);
  ErrorCode initCommunication();

  void onMoveMsg(const RobotMoveType::SharedPtr msg);
  void onToggleHelpPageMsg(const Empty::SharedPtr msg);
  void onToggleDebugInfoMsg(const Empty::SharedPtr msg);

  CollectorControllerExternalBridgeOutInterface _outInterface;

  rclcpp::Subscription<RobotMoveType>::SharedPtr _playerActSubscriber;
  rclcpp::Subscription<Empty>::SharedPtr _toggleHelpPageSubscriber;
  rclcpp::Subscription<Empty>::SharedPtr _toggleDebugInfoSubscriber;

  rclcpp::Publisher<Empty>::SharedPtr _playerEnableInputPublisher;
  rclcpp::Publisher<Empty>::SharedPtr _shutdownControllerPublisher;

  ControllerStatus _controllerStatus = ControllerStatus::IDLE;
};

#endif /* ROBO_COLLECTOR_GUI_COLLECTORCONTROLLEREXTERNALBRIDGE_H_ */
