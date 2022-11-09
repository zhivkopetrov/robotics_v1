#ifndef ROBO_CLEANER_GUI_CLEANERCONTROLLEREXTERNALBRIDGE_H_
#define ROBO_CLEANER_GUI_CLEANERCONTROLLEREXTERNALBRIDGE_H_

//System headers
#include <thread>

//Other libraries headers
#include <rclcpp/node.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include "robo_cleaner_interfaces/srv/query_initial_robot_state.hpp"
#include "robo_cleaner_interfaces/srv/query_battery_status.hpp"
#include "robo_cleaner_interfaces/srv/charge_battery.hpp"
#include "robo_cleaner_interfaces/msg/user_authenticate.hpp"
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
  SetDebugMsgCb setDebugMsgCb;
  SetUserDataCb setUserDataCb;
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

  void publishRobotMoveCounter() const;

  void resetControllerStatus();

private:
  ErrorCode initOutInterface(
      const CleanerControllerExternalBridgeOutInterface &outInterface);
  ErrorCode initCommunication();

  using Empty = std_msgs::msg::Empty;
  using Int32 = std_msgs::msg::Int32;
  using String = std_msgs::msg::String;
  using UserAuthenticate = robo_cleaner_interfaces::msg::UserAuthenticate;
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

  void onUserAuthenticateMsg(const UserAuthenticate::SharedPtr msg);
  void onToggleHelpPageMsg(const Empty::SharedPtr msg);
  void onToggleDebugInfoMsg(const Empty::SharedPtr msg);
  void onDebugMsg(const String::SharedPtr msg);

  CleanerControllerExternalBridgeOutInterface _outInterface;

  rclcpp_action::Server<RobotMove>::SharedPtr _moveActionServer;

  rclcpp::Service<QueryBatteryStatus>::SharedPtr _batteryStatusService;
  rclcpp::Service<QueryInitialRobotState>::SharedPtr _initialRobotStateService;
  rclcpp::Service<ChargeBattery>::SharedPtr _chargeBatteryService;

  rclcpp::Publisher<Empty>::SharedPtr _shutdownControllerPublisher;
  rclcpp::Publisher<Empty>::SharedPtr _fieldMapReveleadedPublisher;
  rclcpp::Publisher<Empty>::SharedPtr _fieldMapCleanedPublisher;
  rclcpp::Publisher<Int32>::SharedPtr _robotMoveCounterPublisher;

  rclcpp::Subscription<UserAuthenticate>::SharedPtr _userAuthenticateSubscriber;
  rclcpp::Subscription<Empty>::SharedPtr _toggleHelpPageSubscriber;
  rclcpp::Subscription<Empty>::SharedPtr _toggleDebugInfoSubscriber;
  rclcpp::Subscription<String>::SharedPtr _setDebugMsgSubscriber;

  //Create different callbacks groups for publishers and subscribers
  //so they can be executed in parallel
  const rclcpp::CallbackGroup::SharedPtr _subscriberCallbackGroup =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  const rclcpp::CallbackGroup::SharedPtr _publishersCallbackGroup =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  //temporary change Reentrant callback group to MutuallyExclusive
  //in order to resolve internal ROS2 implementation crash
  //TODO: figure out why crash is happening only inside the VM
  //and not on the host OS
  const rclcpp::CallbackGroup::SharedPtr _actionsCallbackGroup =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  ControllerStatus _controllerStatus = ControllerStatus::IDLE;
};

#endif /* ROBO_CLEANER_GUI_CLEANERCONTROLLEREXTERNALBRIDGE_H_ */
