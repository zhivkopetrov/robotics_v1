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
#include "robo_cleaner_gui/helpers/EnergyHandler.h"

using namespace std::placeholders;

CleanerControllerExternalBridge::CleanerControllerExternalBridge()
    : Node("CleanerControllerExternalBridge") {

}

CleanerControllerExternalBridge::~CleanerControllerExternalBridge() noexcept {
  _shutdownControllerPublisher->publish(Empty());
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

//called by the main thread
void CleanerControllerExternalBridge::publishShutdownController() {
  _controllerStatus = ControllerStatus::SHUTTING_DOWN;
  _shutdownControllerPublisher->publish(Empty());
}

//called by the update thread
void CleanerControllerExternalBridge::publishFieldMapRevealed() {
  _outInterface.solutionValidator->fieldMapRevealed();
  _fieldMapReveleadedPublisher->publish(Empty());
}

//called by the update thread
void CleanerControllerExternalBridge::publishFieldMapCleaned() {
  _outInterface.solutionValidator->fieldMapCleaned();
  _fieldMapCleanedPublisher->publish(Empty());
}

//called by the update thread
void CleanerControllerExternalBridge::publishRobotMoveCounter() const {
  Int32 msg;
  msg.data = _outInterface.solutionValidator->getTotalRobotMovesCounter();
  _robotMoveCounterPublisher->publish(msg);
}

void CleanerControllerExternalBridge::resetControllerStatus() {
  const auto f = [this]() {
    if (ControllerStatus::SHUTTING_DOWN == _controllerStatus) {
      return;
    }

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

  if (nullptr == _outInterface.toggleHelpPageCb) {
    LOGERR("Error, nullptr provided for ToggleHelpPageCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.toggleDebugInfoCb) {
    LOGERR("Error, nullptr provided for ToggleDebugInfoCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.setDebugMsgCb) {
    LOGERR("Error, nullptr provided for SetDebugMsgCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.setUserDataCb) {
    LOGERR("Error, nullptr provided for SetUserDataCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.startGameLostAnimCb) {
    LOGERR("Error, nullptr provided for StartGameLostAnimCb");
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

  if (nullptr == _outInterface.reportInsufficientEnergyCb) {
    LOGERR("Error, nullptr provided for ReportInsufficientEnergyCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.cancelFeedbackReportingCb) {
    LOGERR("Error, nullptr provided for CancelFeedbackReportingCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.energyHandler) {
    LOGERR("Error, nullptr provided for energyHandler");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.solutionValidator) {
    LOGERR("Error, nullptr provided for SolutionValidator");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode CleanerControllerExternalBridge::initCommunication() {
  constexpr size_t queueSize = 10;
  const rclcpp::QoS qos(queueSize);

  rclcpp::SubscriptionOptions subsriptionOptions;
  subsriptionOptions.callback_group = _subscriberCallbackGroup;

  rclcpp::PublisherOptions publisherOptions;
  publisherOptions.callback_group = _publishersCallbackGroup;

  _shutdownControllerPublisher = create_publisher<Empty>(
      SHUTDOWN_CONTROLLER_TOPIC, qos, publisherOptions);

  _fieldMapReveleadedPublisher = create_publisher<Empty>(
      FIELD_MAP_REVEALED_TOPIC, qos, publisherOptions);

  _fieldMapCleanedPublisher = create_publisher<Empty>(FIELD_MAP_CLEANED_TOPIC,
      qos, publisherOptions);

  _robotMoveCounterPublisher = create_publisher<Int32>(ROBOT_MOVE_COUNTER_TOPIC,
        qos, publisherOptions);

  _batteryStatusService = create_service<QueryBatteryStatus>(
      QUERY_BATTERY_STATUS_SERVICE,
      std::bind(&CleanerControllerExternalBridge::handleBatteryStatusService,
          this, _1, _2), rmw_qos_profile_services_default,
      _subscriberCallbackGroup);

  _initialRobotStateService = create_service<QueryInitialRobotState>(
      QUERY_INITIAL_ROBOT_STATE_SERVICE,
      std::bind(
          &CleanerControllerExternalBridge::handleInitialRobotStateService,
          this, _1, _2), rmw_qos_profile_services_default,
      _subscriberCallbackGroup);

  _chargeBatteryService = create_service<ChargeBattery>(CHARGE_BATTERY_SERVICE,
      std::bind(&CleanerControllerExternalBridge::handleChargeBatteryService,
          this, _1, _2), rmw_qos_profile_services_default,
      _subscriberCallbackGroup);

  _moveActionServer = rclcpp_action::create_server<RobotMove>(this,
      ROBOT_MOVE_ACTION,
      std::bind(&CleanerControllerExternalBridge::handleMoveGoal, this, _1, _2),
      std::bind(&CleanerControllerExternalBridge::handleMoveCancel, this, _1),
      std::bind(&CleanerControllerExternalBridge::handleMoveAccepted, this, _1),
      rcl_action_server_get_default_options(), _actionsCallbackGroup);

  _userAuthenticateSubscriber = create_subscription<UserAuthenticate>(
      USER_AUTHENTICATE_TOPIC, qos,
      std::bind(&CleanerControllerExternalBridge::onUserAuthenticateMsg, this,
          _1), subsriptionOptions);

  _toggleHelpPageSubscriber = create_subscription<Empty>(TOGGLE_HELP_PAGE_TOPIC,
      qos,
      std::bind(&CleanerControllerExternalBridge::onToggleHelpPageMsg, this,
          _1), subsriptionOptions);

  _toggleDebugInfoSubscriber = create_subscription<Empty>(
      TOGGLE_DEBUG_INFO_TOPIC, qos,
      std::bind(&CleanerControllerExternalBridge::onToggleDebugInfoMsg, this,
          _1), subsriptionOptions);

  _setDebugMsgSubscriber = create_subscription<String>(DEBUG_MSG_TOPIC, qos,
      std::bind(&CleanerControllerExternalBridge::onDebugMsg, this, _1),
      subsriptionOptions);

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
          LOGR("Rejecting goal with uuid: %s because another one is already "
               "active", rclcpp_action::to_string(uuid).c_str());
          response = rclcpp_action::GoalResponse::REJECT;
          return;
        }
        if (ControllerStatus::SHUTTING_DOWN == _controllerStatus) {
          LOGR("Rejecting goal with uuid: %s because controller is shutting "
               "down", rclcpp_action::to_string(uuid).c_str());
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
    //First cancel the robot more to initiate state rollback
    _outInterface.robotActInterface.cancelRobotMove();

    //then cancel feedback reporting and process the goal handle canceling state
    _outInterface.cancelFeedbackReportingCb();
  };
  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);

  return rclcpp_action::CancelResponse::ACCEPT;
}

void CleanerControllerExternalBridge::handleMoveAccepted(
    const std::shared_ptr<GoalHandleRobotMove> goalHandle) {
  const auto goal = goalHandle->get_goal();
  const MoveType moveType = getMoveType(goal->robot_move_type.move_type);
  const auto f = [this, moveType]() {
    _outInterface.solutionValidator->increaseTotalRobotMovesCounter(1);
    publishRobotMoveCounter();

    const auto [success, majorError, penaltyTurns] =
        _outInterface.energyHandler->initiateMove();
    if (majorError) {
      handleMajorError();
      return;
    }

    if (!success) {
      _outInterface.energyHandler->performPenaltyChange();
      _outInterface.solutionValidator->increaseTotalRobotMovesCounter(
          penaltyTurns);
      _outInterface.reportInsufficientEnergyCb(penaltyTurns);
      publishRobotMoveCounter();
      return;
    }

    _outInterface.robotActInterface.actCb(moveType);
    const char approachMarker =
        _outInterface.solutionValidator->getApproachingTileMarker(moveType);
    _outInterface.reportRobotStartingActCb(moveType, approachMarker);
  };
  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);

  // return quickly to avoid blocking the executor
  _outInterface.acceptGoalCb(goalHandle);
}

void CleanerControllerExternalBridge::handleBatteryStatusService(
    [[maybe_unused]]const std::shared_ptr<QueryBatteryStatus::Request> request,
    std::shared_ptr<QueryBatteryStatus::Response> response) {
  const auto f = [this, &response]() {
    const auto [maxMoves, movesLeft] =
        _outInterface.energyHandler->queryBatteryStatus();
    response->battery_status.max_moves_on_full_energy = maxMoves;
    response->battery_status.moves_left = movesLeft;
  };

  _outInterface.invokeActionEventCb(f, ActionEventType::BLOCKING);
}

void CleanerControllerExternalBridge::handleInitialRobotStateService(
    [[maybe_unused]]const std::shared_ptr<QueryInitialRobotState::Request> request,
    std::shared_ptr<QueryInitialRobotState::Response> response) {
  const auto f = [this, &response]() {
    InitialRobotState initialRobotState;
    const auto [success, majorError] =
        _outInterface.solutionValidator->queryInitialRobotPos(initialRobotState,
            response->error_reason);
    response->success = success && !majorError;
    if (majorError) {
      handleMajorError();
      return;
    }

    response->initial_robot_state.robot_dir = getRobotDirectionField(
        initialRobotState.robotDir);
    response->initial_robot_state.robot_tile = initialRobotState.robotTile;

    const auto [maxMoves, movesLeft] =
        _outInterface.energyHandler->queryBatteryStatus();
    response->initial_robot_state.battery_status.max_moves_on_full_energy =
        maxMoves;
    response->initial_robot_state.battery_status.moves_left = movesLeft;
  };

  _outInterface.invokeActionEventCb(f, ActionEventType::BLOCKING);
}

void CleanerControllerExternalBridge::handleChargeBatteryService(
    const std::shared_ptr<ChargeBattery::Request> request,
    std::shared_ptr<ChargeBattery::Response> response) {
  //TODO add another state to controller status - CHARGING
  //reject charging if controller is active
  //reject movement if controller is charging

  //TODO-2 add a callback to panelHandler anim modification end
  //attach reset controller status on that callback

  const auto f = [this, &request, &response]() {
    const bool isAtChargingStation =
        _outInterface.solutionValidator->isRobotAtChargingStation();
    if (!isAtChargingStation) {
      response->success = false;
      response->error_reason =
          "Robot is not at charging station. Finishing turn.";
      _outInterface.solutionValidator->increaseTotalRobotMovesCounter(1);
      publishRobotMoveCounter();
      return;
    }

    const ChargeDuration chargeDuration =
        (ChargeBattery::Request::CHARGE_UNTIL_FULL == request->charge_turns) ?
            ChargeDuration::UNTIL_FULL : ChargeDuration::TURN_BASED;

    const ChargeOutcome outcome = _outInterface.energyHandler->charge(
        chargeDuration, request->charge_turns);
    response->success = outcome.success;
    response->error_reason = outcome.errorReason;
    response->turns_spend_charging = outcome.turnsSpendCharging;
    response->battery_status.max_moves_on_full_energy =
        outcome.batteryStatus.maxMovesOnFullEnergy;
    response->battery_status.moves_left = outcome.batteryStatus.movesLeft;

    _outInterface.solutionValidator->increaseTotalRobotMovesCounter(
        outcome.turnsSpendCharging);
    publishRobotMoveCounter();
  };

  _outInterface.invokeActionEventCb(f, ActionEventType::BLOCKING);
}

void CleanerControllerExternalBridge::handleMajorError() {
  _controllerStatus = ControllerStatus::SHUTTING_DOWN;
  _outInterface.startGameLostAnimCb();
}

void CleanerControllerExternalBridge::onUserAuthenticateMsg(
    const UserAuthenticate::SharedPtr msg) {
  const UserData data = { .user = msg->user, .repository = msg->repository,
      .commitSha = msg->commit_sha };

  const auto f = [this, data]() {
    _outInterface.setUserDataCb(data);
  };
  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

void CleanerControllerExternalBridge::onToggleHelpPageMsg(
    [[maybe_unused]]const Empty::SharedPtr msg) {
  const auto f = [this]() {
    _outInterface.toggleHelpPageCb();
  };

  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

void CleanerControllerExternalBridge::onToggleDebugInfoMsg(
    [[maybe_unused]]const Empty::SharedPtr msg) {
  const auto f = [this]() {
    _outInterface.toggleDebugInfoCb();
  };

  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

void CleanerControllerExternalBridge::onDebugMsg(const String::SharedPtr msg) {
  const auto f = [this, msg]() {
    _outInterface.setDebugMsgCb(msg->data);
  };

  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

