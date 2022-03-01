//Corresponding header
#include "robo_miner_gui/external_api/MinerControllerExternalBridge.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "robo_miner_common/defines/RoboMinerTopics.h"
#include "robo_miner_common/message_helpers/RoboMinerMessageHelpers.h"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_miner_gui/helpers/MovementWatcher.h"
#include "robo_miner_gui/helpers/SolutionValidator.h"

using namespace std::literals;

MinerControllerExternalBridge::MinerControllerExternalBridge()
    : Node("MinerControllerExternalBridge") {

}

int32_t MinerControllerExternalBridge::init(
    const MinerControllerExternalBridgeOutInterface &interface) {
  _outInterface = interface;
  if (nullptr == _outInterface.invokeActionEventCb) {
    LOGERR("Error, nullptr provided for InvokeActionEventCb");
    return FAILURE;
  }

  if (nullptr == _outInterface.robotActCb) {
    LOGERR("Error, nullptr provided for RobotActCb");
    return FAILURE;
  }

  if (nullptr == _outInterface.systemShutdownCb) {
    LOGERR("Error, nullptr provided for SystemShutdownCb");
    return FAILURE;
  }

  if (nullptr == _outInterface.movementWatcher) {
    LOGERR("Error, nullptr provided for MovementWatcher");
    return FAILURE;
  }

  if (nullptr == _outInterface.solutionValidator) {
    LOGERR("Error, nullptr provided for SolutionValidator");
    return FAILURE;
  }

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
      LONGEST_SEQUENCE_VALIDATE_SERVICE,
      std::bind(&MinerControllerExternalBridge::handleFieldMapValidateService,
          this, _1, _2));

  _longestSequenceValidateService = create_service<LongestSequenceValidate>(
      LONGEST_SEQUENCE_VALIDATE_SERVICE,
      std::bind(
          &MinerControllerExternalBridge::handleLongestSequenceValidateService,
          this, _1, _2));

  return SUCCESS;
}

void MinerControllerExternalBridge::publishShutdownController() {
  _shutdownControllerPublisher->publish(Empty());

  const auto f = [this]() {
    _outInterface.systemShutdownCb();
  };
  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

void MinerControllerExternalBridge::publishFieldMapRevealed() {
  _fieldMapReveleadedPublisher->publish(Empty());
}

void MinerControllerExternalBridge::handleRobotMoveService(
    const std::shared_ptr<RobotMove::Request> request,
    std::shared_ptr<RobotMove::Response> response) {
  const auto moveType = getMoveType(request->robot_move_type.move_type);
  if (MoveType::UNKNOWN == moveType) {
    LOGERR("Error, received unsupported MoveType: %d", getEnumValue(moveType));
    response->success = false;
    response->error_reason = "Invalid arguments. Unsupported 'move_type' value";
    return;
  }

  const auto f = [this, moveType]() {
    _outInterface.robotActCb(moveType);
  };
  _outInterface.invokeActionEventCb(f, ActionEventType::BLOCKING);

  MoveOutcome outcome;
  SurroundingTiles surroundingTiles;
  const auto success = _outInterface.movementWatcher->waitForChange(5000ms,
      outcome, surroundingTiles);
  if (!success) {
    response->success = false;
    response->error_reason = "Service timed out after 5000ms";
    return;
  }

  if (MoveOutcome::COLLISION == outcome) {
    response->success = false;
    response->error_reason = "Move resulted in collision";
    return;
  }

  response->success = true;
  response->surrounding_tiles = surroundingTiles;
}

void MinerControllerExternalBridge::handleFieldMapValidateService(
    const std::shared_ptr<FieldMapValidate::Request> request,
    [[maybe_unused]]std::shared_ptr<FieldMapValidate::Response> response) {
  const auto& [rows, cols, data] = request->field_map;

  response->success = _outInterface.solutionValidator->validateFieldMap(data,
      rows, cols, response->error_reason);
  if (response->success) {
    const auto f = [this]() {
      _outInterface.startAchievementWonAnimCb(Achievement::SINGLE_STAR);
    };
    _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
  }
}

void MinerControllerExternalBridge::handleLongestSequenceValidateService(
    const std::shared_ptr<LongestSequenceValidate::Request> request,
    std::shared_ptr<LongestSequenceValidate::Response> response) {
  CrystalSequence sequence;
  sequence.reserve(request->sequence_points.size());
  for (const auto &point : request->sequence_points) {
    sequence.emplace_back(point.row, point.col);
  }

  response->success = _outInterface.solutionValidator->validateLongestSequence(
      sequence, response->error_reason);
  if (response->success) {
    const auto f = [this]() {
      _outInterface.startAchievementWonAnimCb(Achievement::DOUBLE_STAR);
    };
    _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
  }
}

