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
  SystemShutdownCb systemShutdownCb;
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

  void onMoveMsg(const RobotMoveType::SharedPtr msg);

  CollectorControllerExternalBridgeOutInterface _outInterface;

  rclcpp::Subscription<RobotMoveType>::SharedPtr _playerActSubscriber;
  rclcpp::Publisher<Empty>::SharedPtr _playerEnableInputPublisher;
  rclcpp::Publisher<Empty>::SharedPtr _shutdownControllerPublisher;
};

#endif /* ROBO_COLLECTOR_GUI_COLLECTORCONTROLLEREXTERNALBRIDGE_H_ */
