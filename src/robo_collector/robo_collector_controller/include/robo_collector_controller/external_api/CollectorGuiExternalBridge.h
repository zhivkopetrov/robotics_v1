#ifndef ROBO_COLLECTOR_CONTROLLER_COLLECTORGUIEXTERNALBRIDGE_H_
#define ROBO_COLLECTOR_CONTROLLER_COLLECTORGUIEXTERNALBRIDGE_H_

//System headers

//Other libraries headers
#include <rclcpp/node.hpp>
#include "std_msgs/msg/empty.hpp"
#include "robo_collector_interfaces/msg/robot_move_type.hpp"
#include "robo_collector_common/defines/RoboCollectorFunctionalDefines.h"
#include "game_engine/defines/ActionEventDefines.h"
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations

struct CollectorGuiExternalBridgeOutInterface {
  InvokeActionEventCb invokeActionEventCb;
  EnablePlayerInputCb enablePlayerInputCb;
  SystemShutdownCb systemShutdownCb;
};

class CollectorGuiExternalBridge: public rclcpp::Node {
public:
  CollectorGuiExternalBridge();

  ErrorCode init(const CollectorGuiExternalBridgeOutInterface &interface);

  void publishToggleDebugInfo() const;
  void publishToggleHelpPage() const;
  void publishRobotAct(MoveType moveType) const;

private:
  using RobotMoveType = robo_collector_interfaces::msg::RobotMoveType;
  using Empty = std_msgs::msg::Empty;

  ErrorCode initOutInterface(
      const CollectorGuiExternalBridgeOutInterface &outInterface);
  ErrorCode initCommunication();

  void onEnableRobotTurnMsg(const Empty::SharedPtr msg);
  void onControllerShutdownMsg(const Empty::SharedPtr msg);

  CollectorGuiExternalBridgeOutInterface _outInterface;
  rclcpp::Publisher<RobotMoveType>::SharedPtr _robotActPublisher;
  rclcpp::Publisher<Empty>::SharedPtr _toggleHelpPagePublisher;
  rclcpp::Publisher<Empty>::SharedPtr _toggleDebugInfoPublisher;

  rclcpp::Subscription<Empty>::SharedPtr _enableRobotTurnSubscription;
  rclcpp::Subscription<Empty>::SharedPtr _shutdownControllerSubscription;
};

#endif /* ROBO_COLLECTOR_CONTROLLER_COLLECTORGUIEXTERNALBRIDGE_H_ */
