//Corresponding header
#include "robo_miner_gui/external_api/MinerControllerExternalBridge.h"

//System headers

//Other libraries headers
#include "robo_miner_common/defines/RoboMinerTopics.h"
#include "robo_miner_common/message_helpers/RoboMinerMessageHelpers.h"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/Log.h"

//Own components headers
#include "robo_miner_gui/helpers/MovementWatcher.h"
#include "robo_miner_gui/helpers/SolutionValidator.h"

using namespace std::literals;

MinerControllerExternalBridge::MinerControllerExternalBridge()
    : Node("MinerControllerExternalBridge") {

}

MinerControllerExternalBridge::~MinerControllerExternalBridge() noexcept {
  _shutdownControllerPublisher->publish(Empty());
}

ErrorCode MinerControllerExternalBridge::init(
    const MinerControllerExternalBridgeOutInterface &interface) {
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

void MinerControllerExternalBridge::deinit() {
  _outInterface.movementWatcher->terminateAction();
}

//called from the main thread
void MinerControllerExternalBridge::publishShutdownController() {
  _controllerStatus = ControllerStatus::SHUTTING_DOWN;
  _shutdownControllerPublisher->publish(Empty());
}

//called from the main thread
void MinerControllerExternalBridge::publishFieldMapRevealed() {
  _outInterface.solutionValidator->fieldMapRevealed();
  _fieldMapReveleadedPublisher->publish(Empty());
}

ErrorCode MinerControllerExternalBridge::initOutInterface(
    const MinerControllerExternalBridgeOutInterface &outInterface) {
  _outInterface = outInterface;
  if (nullptr == _outInterface.invokeActionEventCb) {
    LOGERR("Error, nullptr provided for InvokeActionEventCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.robotActCb) {
    LOGERR("Error, nullptr provided for RobotActCb");
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

  if (nullptr == _outInterface.startAchievementWonAnimCb) {
    LOGERR("Error, nullptr provided for StartAchievementWonAnimCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.startGameLostAnimCb) {
    LOGERR("Error, nullptr provided for StartGameLostAnimCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.tileReleavedCb) {
    LOGERR("Error, nullptr provided for TileReleavedCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.revealFogOfWarTilesCb) {
    LOGERR("Error, nullptr provided for RevealFogOfWarTilesCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.crystalMinedCb) {
    LOGERR("Error, nullptr provided for CrystalMinedCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.movementWatcher) {
    LOGERR("Error, nullptr provided for MovementWatcher");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.solutionValidator) {
    LOGERR("Error, nullptr provided for SolutionValidator");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode MinerControllerExternalBridge::initCommunication() {
  using namespace std::placeholders;
  constexpr size_t queueSize = 10;
  const rclcpp::QoS qos(queueSize);

  const rmw_qos_profile_t deafultQosProfile{};

  //Create different callbacks groups for publishers and subscribers
  //so they can be executed in parallel
  const rclcpp::CallbackGroup::SharedPtr subscriberCallbackGroup =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions subsriptionOptions;
  subsriptionOptions.callback_group = subscriberCallbackGroup;

  const rclcpp::CallbackGroup::SharedPtr publishersCallbackGroup =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::PublisherOptions publisherOptions;
  publisherOptions.callback_group = publishersCallbackGroup;

  _shutdownControllerPublisher = create_publisher<Empty>(
      SHUTDOWN_CONTROLLER_TOPIC, qos, publisherOptions);

  _fieldMapReveleadedPublisher = create_publisher<Empty>(
      FIELD_MAP_REVEALED_TOPIC, qos, publisherOptions);

  _initialRobotPosService = create_service<QueryInitialRobotPosition>(
      QUERY_INITIAL_ROBOT_POSITION_SERVICE,
      std::bind(&MinerControllerExternalBridge::handleInitialRobotPosService,
          this, _1, _2), deafultQosProfile);

  _robotMoveService = create_service<RobotMove>(ROBOT_MOVE_SERVICE,
      std::bind(&MinerControllerExternalBridge::handleRobotMoveService, this,
          _1, _2), deafultQosProfile, subscriberCallbackGroup);

  _fieldMapValidateService = create_service<FieldMapValidate>(
      FIELD_MAP_VALIDATE_SERVICE,
      std::bind(&MinerControllerExternalBridge::handleFieldMapValidateService,
          this, _1, _2), deafultQosProfile, subscriberCallbackGroup);

  _longestSequenceValidateService = create_service<LongestSequenceValidate>(
      LONGEST_SEQUENCE_VALIDATE_SERVICE,
      std::bind(
          &MinerControllerExternalBridge::handleLongestSequenceValidateService,
          this, _1, _2), deafultQosProfile, subscriberCallbackGroup);

  _activateMiningValidateService = create_service<ActivateMiningValidate>(
      ACTIVATE_MINING_VALIDATE_SERVICE,
      std::bind(
          &MinerControllerExternalBridge::handleActivateMiningValidateService,
          this, _1, _2), deafultQosProfile, subscriberCallbackGroup);

  _userAuthenticateSubscriber = create_subscription<UserAuthenticate>(
      USER_AUTHENTICATE_TOPIC, qos,
      std::bind(&MinerControllerExternalBridge::onUserAuthenticateMsg, this,
          _1), subsriptionOptions);

  _toggleHelpPageSubscriber = create_subscription<Empty>(TOGGLE_HELP_PAGE_TOPIC,
      qos,
      std::bind(&MinerControllerExternalBridge::onToggleHelpPageMsg, this, _1),
      subsriptionOptions);

  _toggleDebugInfoSubscriber = create_subscription<Empty>(
      TOGGLE_DEBUG_INFO_TOPIC, qos,
      std::bind(&MinerControllerExternalBridge::onToggleDebugInfoMsg, this,
          _1), subsriptionOptions);

  _setDebugMsgSubscriber = create_subscription<String>(DEBUG_MSG_TOPIC, qos,
      std::bind(&MinerControllerExternalBridge::onDebugMsg, this, _1),
      subsriptionOptions);

  return ErrorCode::SUCCESS;
}

void MinerControllerExternalBridge::handleInitialRobotPosService(
    [[maybe_unused]]const std::shared_ptr<QueryInitialRobotPosition::Request> request,
    std::shared_ptr<QueryInitialRobotPosition::Response> response) {
  const auto f = [this, &response]() {
    InitialRobotPos initialRobotPos;
    const auto [success, majorError] =
        _outInterface.solutionValidator->queryInitialRobotPos(initialRobotPos,
            response->robot_position_response.error_reason);

    response->robot_position_response.success = success && !majorError;
    if (majorError) {
      handleMajorError();
      return;
    }

    response->robot_position_response.surrounding_tiles =
        initialRobotPos.surroundingTiles;
    response->robot_position_response.robot_dir = getRobotDirectionField(
        initialRobotPos.robotDir);
    response->robot_initial_tile = initialRobotPos.robotTile;
  };
  _outInterface.invokeActionEventCb(f, ActionEventType::BLOCKING);
}

void MinerControllerExternalBridge::handleRobotMoveService(
    const std::shared_ptr<RobotMove::Request> request,
    std::shared_ptr<RobotMove::Response> response) {
  response->robot_position_response.success = true;

  const auto moveType = getMoveType(request->robot_move_type.move_type);
  if (MoveType::UNKNOWN == moveType) {
    LOGERR("Error, received unsupported MoveType: %d", getEnumValue(moveType));
    response->robot_position_response.success = false;
    response->robot_position_response.error_reason =
        "Invalid arguments. Unsupported 'move_type' value";
    return;
  }

  const auto f = [this, &response, moveType]() {
    if (ControllerStatus::ACTIVE == _controllerStatus) {
      response->robot_position_response.success = false;
      auto& str = response->robot_position_response.error_reason;
      str = "Rejecting Move Service with type: [";
      str.append(std::to_string(getEnumValue(moveType))).
          append("], because another one is already active");
      LOGR("%s", response->robot_position_response.error_reason.c_str());
      return;
    }
    if (ControllerStatus::SHUTTING_DOWN == _controllerStatus) {
      response->robot_position_response.success = false;
      auto& str = response->robot_position_response.error_reason;
      str = "Rejecting Move Service with type: [";
      str.append(std::to_string(getEnumValue(moveType))).
          append("], because controller is shutting down");
      LOGR("%s", response->robot_position_response.error_reason.c_str());
      return;
    }

    _controllerStatus = ControllerStatus::ACTIVE;
  };
  _outInterface.invokeActionEventCb(f, ActionEventType::BLOCKING);
  if (!response->robot_position_response.success) {
    return;
  }

  const auto f2 = [this, moveType]() {
    _outInterface.robotActCb(moveType);
  };
  _outInterface.invokeActionEventCb(f2, ActionEventType::BLOCKING);

  MovementWatchOutcome outcome;
  const auto success = _outInterface.movementWatcher->waitForChange(5000ms,
      outcome);

  if (outcome.actionTerminated) {
    LOGY("Move action terminated. Controller is shutting down, no further "
         "actions will be taken");
    return;
  }

  response->robot_position_response.robot_dir = getRobotDirectionField(
      outcome.robotDir);

  //reset controller before timing out
  const auto f3 = [this]() {
    _controllerStatus = ControllerStatus::IDLE;
  };
  _outInterface.invokeActionEventCb(f3, ActionEventType::NON_BLOCKING);

  if (!success) {
    response->robot_position_response.success = false;
    response->robot_position_response.error_reason =
        "Service timed out after 5000ms";
    return;
  }

  if (MoveOutcome::COLLISION == outcome.moveOutcome) {
    response->robot_position_response.success = false;
    response->robot_position_response.error_reason =
        "Move resulted in collision";
    return;
  }

  response->robot_position_response.success = true;
  response->robot_position_response.surrounding_tiles =
      outcome.surroundingTiles;

  const auto f4 = [this, outcome]() {
    if (_outInterface.solutionValidator->isMiningActive()) {
      handleMiningMove(outcome.robotPos);
    } else {
      handleNormalMove(outcome.robotPos);
    }
  };
  _outInterface.invokeActionEventCb(f4, ActionEventType::NON_BLOCKING);
}

void MinerControllerExternalBridge::handleFieldMapValidateService(
    const std::shared_ptr<FieldMapValidate::Request> request,
    std::shared_ptr<FieldMapValidate::Response> response) {
  const auto f = [&, this]() {
    const auto& [rows, cols, data] = request->field_map;

    const auto [success, majorError] =
        _outInterface.solutionValidator->validateFieldMap(data, rows, cols,
            response->error_reason);
    response->success = success && !majorError;
    if (majorError) {
      handleMajorError();
      return;
    }

    if (response->success) {
      _outInterface.revealFogOfWarTilesCb();
      _outInterface.startAchievementWonAnimCb(Achievement::SINGLE_STAR);
    }
  };

  _outInterface.invokeActionEventCb(f, ActionEventType::BLOCKING);
}

void MinerControllerExternalBridge::handleLongestSequenceValidateService(
    const std::shared_ptr<LongestSequenceValidate::Request> request,
    std::shared_ptr<LongestSequenceValidate::Response> response) {
  CrystalSequence sequence;
  sequence.reserve(request->sequence_points.size());
  for (const auto &point : request->sequence_points) {
    sequence.emplace_back(point.row, point.col);
  }

  const auto f = [this, &response, &sequence]() {
    const auto [success, majorError] =
        _outInterface.solutionValidator->validateLongestSequence(sequence,
            response->error_reason);
    response->success = success && !majorError;
    if (majorError) {
      handleMajorError();
      return;
    }

    if (response->success) {
      _outInterface.startAchievementWonAnimCb(Achievement::DOUBLE_STAR);
    }
  };

  _outInterface.invokeActionEventCb(f, ActionEventType::BLOCKING);
}

void MinerControllerExternalBridge::handleActivateMiningValidateService(
    [[maybe_unused]]const std::shared_ptr<ActivateMiningValidate::Request> request,
    std::shared_ptr<ActivateMiningValidate::Response> response) {
  const auto f = [this, &response]() {
    const auto [success, majorError] =
        _outInterface.solutionValidator->validateActivateMining(
            response->error_reason);
    response->success = success && !majorError;
    if (majorError) {
      handleMajorError();
      return;
    }

    if (success) {
      _outInterface.crystalMinedCb();
    }
  };

  _outInterface.invokeActionEventCb(f, ActionEventType::BLOCKING);
}

void MinerControllerExternalBridge::onUserAuthenticateMsg(
    const UserAuthenticate::SharedPtr msg) {
  const UserData data = { .user = msg->user, .repository = msg->repository,
      .commitSha = msg->commit_sha };

  const auto f = [this, data]() {
    _outInterface.setUserDataCb(data);
  };
  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

void MinerControllerExternalBridge::onToggleHelpPageMsg(
    [[maybe_unused]]const Empty::SharedPtr msg) {
  const auto f = [this]() {
    _outInterface.toggleHelpPageCb();
  };

  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

void MinerControllerExternalBridge::onToggleDebugInfoMsg(
    [[maybe_unused]]const Empty::SharedPtr msg) {
  const auto f = [this]() {
    _outInterface.toggleDebugInfoCb();
  };

  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

void MinerControllerExternalBridge::onDebugMsg(const String::SharedPtr msg) {
  const auto f = [this, msg]() {
    _outInterface.setDebugMsgCb(msg->data);
  };

  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

void MinerControllerExternalBridge::handleNormalMove(const FieldPos &robotPos) {
  const auto [success, _] = _outInterface.solutionValidator->handleNormalMove(
      robotPos);

  if (success) {
    _outInterface.tileReleavedCb();
  }
}

void MinerControllerExternalBridge::handleMiningMove(const FieldPos &robotPos) {
  const auto [success, majorError] =
      _outInterface.solutionValidator->handleMiningMove(robotPos);
  if (majorError) {
    handleMajorError();
    return;
  }

  if (success) {
    _outInterface.crystalMinedCb();
  }
}

void MinerControllerExternalBridge::handleMajorError() {
  _controllerStatus = ControllerStatus::SHUTTING_DOWN;
  _outInterface.startGameLostAnimCb();
}

