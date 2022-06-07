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

void MinerControllerExternalBridge::publishShutdownController() {
  _shutdownControllerPublisher->publish(Empty());

  const auto f = [this]() {
    _outInterface.systemShutdownCb();
  };
  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

void MinerControllerExternalBridge::publishFieldMapRevealed() {
  const auto f = [this]() {
    _outInterface.solutionValidator->fieldMapRevealed();
  };
  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);

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

  if (nullptr == _outInterface.systemShutdownCb) {
    LOGERR("Error, nullptr provided for SystemShutdownCb");
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
  constexpr auto queueSize = 10;
  _shutdownControllerPublisher = create_publisher<Empty>(
      SHUTDOWN_CONTROLLER_TOPIC, queueSize);

  _fieldMapReveleadedPublisher = create_publisher<Empty>(
      FIELD_MAP_REVEALED_TOPIC, queueSize);

  _robotMoveService = create_service<RobotMove>(ROBOT_MOVE_SERVICE,
      std::bind(&MinerControllerExternalBridge::handleRobotMoveService, this,
          _1, _2));

  _fieldMapValidateService = create_service<FieldMapValidate>(
      FIELD_MAP_VALIDATE_SERVICE,
      std::bind(&MinerControllerExternalBridge::handleFieldMapValidateService,
          this, _1, _2));

  _longestSequenceValidateService = create_service<LongestSequenceValidate>(
      LONGEST_SEQUENCE_VALIDATE_SERVICE,
      std::bind(
          &MinerControllerExternalBridge::handleLongestSequenceValidateService,
          this, _1, _2));

  _activateMiningValidateService = create_service<ActivateMiningValidate>(
      ACTIVATE_MINING_VALIDATE_SERVICE,
      std::bind(
          &MinerControllerExternalBridge::handleActivateMiningValidateService,
          this, _1, _2));

  return ErrorCode::SUCCESS;
}

void MinerControllerExternalBridge::handleRobotMoveService(
    const std::shared_ptr<RobotMove::Request> request,
    std::shared_ptr<RobotMove::Response> response) {
  response->success = true;

  const auto moveType = getMoveType(request->robot_move_type.move_type);
  if (MoveType::UNKNOWN == moveType) {
    LOGERR("Error, received unsupported MoveType: %d", getEnumValue(moveType));
    response->success = false;
    response->error_reason = "Invalid arguments. Unsupported 'move_type' value";
    return;
  }

  const auto f = [this, &response]() {
    if (ControllerStatus::ACTIVE == _controllerStatus) {
      response->success = false;
      response->error_reason =
          "Rejecting Move Service because another one is already active";
      LOGERR("%s", response->error_reason.c_str());
      return;
    }

    _controllerStatus = ControllerStatus::ACTIVE;
  };
  _outInterface.invokeActionEventCb(f, ActionEventType::BLOCKING);
  if (!response->success) {
    return;
  }

  const auto f2 = [this, moveType]() {
    _outInterface.robotActCb(moveType);
  };
  _outInterface.invokeActionEventCb(f2, ActionEventType::BLOCKING);

  MovementWatchOutcome outcome;
  const auto success = _outInterface.movementWatcher->waitForChange(5000ms,
      outcome);

  const auto f3 = [this]() {
    _controllerStatus = ControllerStatus::IDLE;
  };
  _outInterface.invokeActionEventCb(f3, ActionEventType::NON_BLOCKING);

  if (!success) {
    response->success = false;
    response->error_reason = "Service timed out after 5000ms";
    return;
  }

  if (MoveOutcome::COLLISION == outcome.moveOutcome) {
    response->success = false;
    response->error_reason = "Move resulted in collision";
    return;
  }

  response->success = true;
  response->surrounding_tiles = outcome.surroundingTiles;

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
    response->success = success || majorError;
    if (majorError) {
      _outInterface.startGameLostAnimCb();
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
    response->success = success || majorError;
    if (majorError) {
      _outInterface.startGameLostAnimCb();
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
    response->success = success || majorError;
    if (majorError) {
      _outInterface.startGameLostAnimCb();
      return;
    }

    if (success) {
      _outInterface.crystalMinedCb();
    }
  };

  _outInterface.invokeActionEventCb(f, ActionEventType::BLOCKING);
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
    _outInterface.startGameLostAnimCb();
    return;
  }

  if (success) {
    _outInterface.crystalMinedCb();
  }
}

