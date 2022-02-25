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

  using namespace std::placeholders;
  constexpr auto queueSize = 10;
  _shutdownControllerPublisher = create_publisher<Empty>(
      SHUTDOWN_CONTROLLER_TOPIC, queueSize);

  _robotMoveService = create_service<RobotMove>(ROBOT_MOVE_SERVICE,
      std::bind(&MinerControllerExternalBridge::handleRobotMoveService, this,
          _1, _2));

  return SUCCESS;
}

void MinerControllerExternalBridge::publishShutdownController() {
  _shutdownControllerPublisher->publish(Empty());

  const auto f = [this]() {
    _outInterface.systemShutdownCb();
  };
  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

void MinerControllerExternalBridge::handleRobotMoveService(
    const std::shared_ptr<RobotMove::Request> request,
    std::shared_ptr<RobotMove::Response> response) {
  const auto moveType = getMoveType(request->robot_move_type.move_type);
  if (MoveType::UNKNOWN == moveType) {
    LOGERR("Error, received unsupported MoveType: %d", getEnumValue(moveType));
    response->success = false;
    return;
  }

  response->success = true;
  const auto f = [this, moveType]() {
    _outInterface.robotActCb(moveType);
  };
  _outInterface.invokeActionEventCb(f, ActionEventType::BLOCKING);

  MoveOutcome outcome;
  SurroundingTiles surroundingTiles;
  const auto success = _outInterface.movementWatcher->waitForChange(5000ms,
      outcome, surroundingTiles);
  if (!success || (MoveOutcome::COLLISION == outcome)) {
    response->move_success = false;
    return;
  }

  response->move_success = true;
  response->surrounding_tiles = surroundingTiles;
}

