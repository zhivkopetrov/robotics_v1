//Corresponding header
#include "robo_cleaner_gui/external_api/CleanerControllerExternalBridge.h"

//System headers

//Other libraries headers
#include "robo_cleaner_common/defines/RoboCleanerTopics.h"
#include "robo_cleaner_common/message_helpers/RoboCleanerMessageHelpers.h"
#include "robo_common/defines/RoboCommonDefines.h"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_cleaner_gui/helpers/RoboCleanerSolutionValidator.h"

using namespace std::placeholders;

CleanerControllerExternalBridge::CleanerControllerExternalBridge()
    : Node("CleanerControllerExternalBridge") {

}

ErrorCode CleanerControllerExternalBridge::init(
    const CleanerControllerExternalBridgeOutInterface &interface) {
  if (ErrorCode::SUCCESS != initOutInterface(interface)) {
    LOGERR("Error, initOutInterface() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initCommunication()) {
    LOGERR("Error, initCommunication() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

void CleanerControllerExternalBridge::publishShutdownController() {
  _shutdownControllerPublisher->publish(Empty());

  const auto f = [this]() {
    _outInterface.systemShutdownCb();
  };
  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

void CleanerControllerExternalBridge::publishFieldMapRevealed() {
  _outInterface.solutionValidator->fieldMapRevealed();
  _fieldMapReveleadedPublisher->publish(Empty());
}

void CleanerControllerExternalBridge::publishFieldMapCleaned() {
  _outInterface.solutionValidator->fieldMapCleaned();
  _fieldMapCleanedPublisher->publish(Empty());
}

void CleanerControllerExternalBridge::resetControllerStatus() {
  const auto f = [this]() {
    _controllerStatus = ControllerStatus::IDLE;
  };
  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

ErrorCode CleanerControllerExternalBridge::initOutInterface(
    const CleanerControllerExternalBridgeOutInterface &outInterface) {
  _outInterface = outInterface;
  if (nullptr == _outInterface.invokeActionEventCb) {
    LOGERR("Error, nullptr provided for InvokeActionEventCb");
    return ErrorCode::FAILURE;
  }

  if (!_outInterface.robotActInterface.isValid()) {
    LOGERR("Error, RobotActInterface is not populated");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.systemShutdownCb) {
    LOGERR("Error, nullptr provided for SystemShutdownCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.acceptGoalCb) {
    LOGERR("Error, nullptr provided for AcceptGoalCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.reportRobotStartingActCb) {
    LOGERR("Error, nullptr provided for ReportRobotStartingActCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.cancelFeedbackReportingCb) {
    LOGERR("Error, nullptr provided for CancelFeedbackReportingCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.solutionValidator) {
    LOGERR("Error, nullptr provided for SolutionValidator");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode CleanerControllerExternalBridge::initCommunication() {
  constexpr auto queueSize = 10;
  _shutdownControllerPublisher = create_publisher<Empty>(
      SHUTDOWN_CONTROLLER_TOPIC, queueSize);

  _fieldMapReveleadedPublisher = create_publisher<Empty>(
      FIELD_MAP_REVEALED_TOPIC, queueSize);

  _fieldMapCleanedPublisher = create_publisher<Empty>(FIELD_MAP_CLEANED_TOPIC,
      queueSize);

  _moveActionServer = rclcpp_action::create_server<RobotMove>(this,
      ROBOT_MOVE_ACTION,
      std::bind(&CleanerControllerExternalBridge::handleMoveGoal, this, _1, _2),
      std::bind(&CleanerControllerExternalBridge::handleMoveCancel, this, _1),
      std::bind(&CleanerControllerExternalBridge::handleMoveAccepted, this,
          _1));

  return ErrorCode::SUCCESS;
}

rclcpp_action::GoalResponse CleanerControllerExternalBridge::handleMoveGoal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const RobotMove::Goal> goal) {
  LOG("Received goal request with moveType: %hhd and uuid: %s",
      goal->robot_move_type.move_type, rclcpp_action::to_string(uuid).c_str());

  const auto moveType = getMoveType(goal->robot_move_type.move_type);
  if (MoveType::UNKNOWN == moveType) {
    LOGERR("Error, Rejecting goal with uuid: %s because of unsupported "
        "MoveType", rclcpp_action::to_string(uuid).c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  auto response = rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  const auto f =
      [this, &uuid, &response]() {
        if (ControllerStatus::ACTIVE == _controllerStatus) {
          LOGERR("Error, Rejecting goal with uuid: %s because another one is "
              "already active",
              rclcpp_action::to_string(uuid).c_str());
          response = rclcpp_action::GoalResponse::REJECT;
          return;
        }

        _controllerStatus = ControllerStatus::ACTIVE;
      };
  _outInterface.invokeActionEventCb(f, ActionEventType::BLOCKING);

  return response;
}

rclcpp_action::CancelResponse CleanerControllerExternalBridge::handleMoveCancel(
    const std::shared_ptr<GoalHandleRobotMove> goalHandle) {
  LOG("Received request to cancel goal with uuid: %s, Rolling back robot "
      "position/rotation to previous state",
      rclcpp_action::to_string(goalHandle->get_goal_id()).c_str());

  const auto f = [this]() {
    _controllerStatus = ControllerStatus::IDLE;
    _outInterface.cancelFeedbackReportingCb();
    _outInterface.robotActInterface.cancelRobotMove();
  };
  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);

  return rclcpp_action::CancelResponse::ACCEPT;
}

void CleanerControllerExternalBridge::handleMoveAccepted(
    const std::shared_ptr<GoalHandleRobotMove> goalHandle) {
  const auto goal = goalHandle->get_goal();
  const MoveType moveType = getMoveType(goal->robot_move_type.move_type);
  const auto f = [this, moveType]() {
    _outInterface.robotActInterface.actCb(moveType);
     const char approachMarker =
         _outInterface.solutionValidator->getApproachingTileMarker(moveType);
    _outInterface.reportRobotStartingActCb(moveType, approachMarker);
  };
  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);

  // return quickly to avoid blocking the executor
  _outInterface.acceptGoalCb(goalHandle);
}

