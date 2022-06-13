#ifndef ROBO_CLEANER_GUI_CLEANERCONTROLLEREXTERNALBRIDGE_H_
#define ROBO_CLEANER_GUI_CLEANERCONTROLLEREXTERNALBRIDGE_H_

//System headers
#include <thread>

//Other libraries headers
#include <rclcpp/node.hpp>
#include <std_msgs/msg/empty.hpp>
#include "robo_cleaner_interfaces/srv/query_initial_robot_state.hpp"
#include "robo_cleaner_interfaces/srv/query_battery_status.hpp"
#include "robo_cleaner_interfaces/srv/charge_battery.hpp"
#include "robo_common/layout/entities/robot/helpers/RobotActInterface.h"
#include "game_engine/defines/ActionEventDefines.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "robo_cleaner_gui/defines/RoboCleanerGuiFunctionalDefines.h"

//Forward declarations
class RoboCleanerSolutionValidator;
class EnergyHandler;

struct CleanerControllerExternalBridgeOutInterface {
  InvokeActionEventCb invokeActionEventCb;
  RobotActInterface robotActInterface;
  ToggleHelpPageCb toggleHelpPageCb;
  ToggleDebugInfoCb toggleDebugInfoCb;
  StartGameLostAnimCb startGameLostAnimCb;
  AcceptGoalCb acceptGoalCb;
  ReportRobotStartingActCb reportRobotStartingActCb;
  ReportInsufficientEnergyCb reportInsufficientEnergyCb;
  CancelFeedbackReportingCb cancelFeedbackReportingCb;
  EnergyHandler *energyHandler = nullptr;
  RoboCleanerSolutionValidator *solutionValidator = nullptr;
};

class CleanerControllerExternalBridge: public rclcpp::Node {
public:
  CleanerControllerExternalBridge();
  ~CleanerControllerExternalBridge() noexcept;

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
  using QueryBatteryStatus = robo_cleaner_interfaces::srv::QueryBatteryStatus;
  using QueryInitialRobotState = robo_cleaner_interfaces::srv::QueryInitialRobotState;
  using ChargeBattery = robo_cleaner_interfaces::srv::ChargeBattery;

  enum class ControllerStatus {
    IDLE, ACTIVE, SHUTTING_DOWN
  };

  rclcpp_action::GoalResponse handleMoveGoal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const RobotMove::Goal> goal);

  rclcpp_action::CancelResponse handleMoveCancel(
      const std::shared_ptr<GoalHandleRobotMove> goalHandle);

  void handleMoveAccepted(
      const std::shared_ptr<GoalHandleRobotMove> goalHandle);

  void handleBatteryStatusService(
      const std::shared_ptr<QueryBatteryStatus::Request> request,
      std::shared_ptr<QueryBatteryStatus::Response> response);

  void handleInitialRobotStateService(
      const std::shared_ptr<QueryInitialRobotState::Request> request,
      std::shared_ptr<QueryInitialRobotState::Response> response);

  void handleChargeBatteryService(
      const std::shared_ptr<ChargeBattery::Request> request,
      std::shared_ptr<ChargeBattery::Response> response);

  void handleMajorError();

  void onToggleHelpPageMsg(const Empty::SharedPtr msg);
  void onToggleDebugInfoMsg(const Empty::SharedPtr msg);

  CleanerControllerExternalBridgeOutInterface _outInterface;

  rclcpp_action::Server<RobotMove>::SharedPtr _moveActionServer;

  rclcpp::Service<QueryBatteryStatus>::SharedPtr _batteryStatusService;
  rclcpp::Service<QueryInitialRobotState>::SharedPtr _initialRobotStateService;
  rclcpp::Service<ChargeBattery>::SharedPtr _chargeBatteryService;

  rclcpp::Publisher<Empty>::SharedPtr _shutdownControllerPublisher;
  rclcpp::Publisher<Empty>::SharedPtr _fieldMapReveleadedPublisher;
  rclcpp::Publisher<Empty>::SharedPtr _fieldMapCleanedPublisher;

  rclcpp::Subscription<Empty>::SharedPtr _toggleHelpPageSubscriber;
  rclcpp::Subscription<Empty>::SharedPtr _toggleDebugInfoSubscriber;

  ControllerStatus _controllerStatus = ControllerStatus::IDLE;
};

#endif /* ROBO_CLEANER_GUI_CLEANERCONTROLLEREXTERNALBRIDGE_H_ */
