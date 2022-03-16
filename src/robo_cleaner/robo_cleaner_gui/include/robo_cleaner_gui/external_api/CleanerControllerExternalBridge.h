#ifndef ROBO_CLEANER_GUI_CLEANERCONTROLLEREXTERNALBRIDGE_H_
#define ROBO_CLEANER_GUI_CLEANERCONTROLLEREXTERNALBRIDGE_H_

//System headers

//Other libraries headers
#include <rclcpp/node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/empty.hpp>
#include "robo_cleaner_interfaces/action/robot_move.hpp"
#include "game_engine/defines/ActionEventDefines.h"
#include "robo_common/defines/RoboCommonFunctionalDefines.h"
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations

struct CleanerControllerExternalBridgeOutInterface {
  InvokeActionEventCb invokeActionEventCb;
  RobotActCb robotActCb;
  SystemShutdownCb systemShutdownCb;
};

class CleanerControllerExternalBridge: public rclcpp::Node {
public:
  CleanerControllerExternalBridge();

  ErrorCode init(const CleanerControllerExternalBridgeOutInterface &interface);

  void publishShutdownController();

private:
  ErrorCode initOutInterface(
      const CleanerControllerExternalBridgeOutInterface &outInterface);
  ErrorCode initCommunication();

  using Empty = std_msgs::msg::Empty;
  using RobotMove = robo_cleaner_interfaces::action::RobotMove;
  using GoalHandleRobotMove = rclcpp_action::ServerGoalHandle<RobotMove>;

  rclcpp_action::GoalResponse handleMoveGoal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const RobotMove::Goal> goal);

  rclcpp_action::CancelResponse handleMoveCancel(
      const std::shared_ptr<GoalHandleRobotMove> goalHandle);

  void handleMoveAccepted(
      const std::shared_ptr<GoalHandleRobotMove> goalHandle);

  void executeMove(const std::shared_ptr<GoalHandleRobotMove> goalHandle);

  CleanerControllerExternalBridgeOutInterface _outInterface;

  rclcpp_action::Server<RobotMove>::SharedPtr _moveActionServer;

  rclcpp::Publisher<Empty>::SharedPtr _shutdownControllerPublisher;
  rclcpp::Publisher<Empty>::SharedPtr _fieldMapReveleadedPublisher;
  rclcpp::Publisher<Empty>::SharedPtr _fieldMapCleanedPublisher;

//  rclcpp::Subscription<robo_cleaner_interfaces::msg::RobotMoveType>::SharedPtr _playerDirSubscriber;
};

#endif /* ROBO_CLEANER_GUI_CLEANERCONTROLLEREXTERNALBRIDGE_H_ */
