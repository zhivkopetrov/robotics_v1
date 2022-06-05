#ifndef ROBO_CLEANER_GUI_CLEANERCONTROLLEREXTERNALBRIDGE_H_
#define ROBO_CLEANER_GUI_CLEANERCONTROLLEREXTERNALBRIDGE_H_

//System headers
#include <thread>

//Other libraries headers
#include <rclcpp/node.hpp>
#include <std_msgs/msg/empty.hpp>
#include "robo_cleaner_interfaces/srv/field_map_validate.hpp"
#include "robo_common/layout/entities/robot/helpers/RobotActInterface.h"
#include "game_engine/defines/ActionEventDefines.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "robo_cleaner_gui/defines/RoboCleanerGuiFunctionalDefines.h"

//Forward declarations
class RoboCleanerSolutionValidator;

struct CleanerControllerExternalBridgeOutInterface {
  InvokeActionEventCb invokeActionEventCb;
  RobotActInterface robotActInterface;
  SystemShutdownCb systemShutdownCb;
  AcceptGoalCb acceptGoalCb;
  ReportRobotStartingActCb reportRobotStartingActCb;
  CancelFeedbackReportingCb cancelFeedbackReportingCb;
  RoboCleanerSolutionValidator *solutionValidator = nullptr;
};

class CleanerControllerExternalBridge: public rclcpp::Node {
public:
  CleanerControllerExternalBridge();

  ErrorCode init(const CleanerControllerExternalBridgeOutInterface &interface);

  void publishShutdownController();

  void publishFieldMapRevealed();

  void publishFieldMapCleaned();

  void resetControllerStatus();

private:
  ErrorCode initOutInterface(
      const CleanerControllerExternalBridgeOutInterface &outInterface);
  ErrorCode initCommunication();

  using Empty = std_msgs::msg::Empty;
  using FieldMapValidate = robo_cleaner_interfaces::srv::FieldMapValidate;

  enum class ControllerStatus {
    IDLE, ACTIVE
  };

  rclcpp_action::GoalResponse handleMoveGoal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const RobotMove::Goal> goal);

  rclcpp_action::CancelResponse handleMoveCancel(
      const std::shared_ptr<GoalHandleRobotMove> goalHandle);

  void handleMoveAccepted(
      const std::shared_ptr<GoalHandleRobotMove> goalHandle);

  CleanerControllerExternalBridgeOutInterface _outInterface;

  rclcpp_action::Server<RobotMove>::SharedPtr _moveActionServer;

  rclcpp::Publisher<Empty>::SharedPtr _shutdownControllerPublisher;
  rclcpp::Service<FieldMapValidate>::SharedPtr _fieldMapValidateService;

  rclcpp::Publisher<Empty>::SharedPtr _fieldMapReveleadedPublisher;
  rclcpp::Publisher<Empty>::SharedPtr _fieldMapCleanedPublisher;

  ControllerStatus _controllerStatus = ControllerStatus::IDLE;
};

#endif /* ROBO_CLEANER_GUI_CLEANERCONTROLLEREXTERNALBRIDGE_H_ */
