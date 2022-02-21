//Corresponding header
#include "robo_collector_gui/external_api/CollectorControllerExternalBridge.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

namespace {
//TODO create separate message helper utility file
MoveType getMoveType(const int8_t moveType) {
  using robo_collector_interfaces::msg::RobotMoveType;
  switch (moveType) {
  case RobotMoveType::FORWARD:
    return MoveType::FORWARD;
  case RobotMoveType::ROTATE_LEFT:
    return MoveType::ROTATE_LEFT;
  case RobotMoveType::ROTATE_RIGHT:
    return MoveType::ROTATE_RIGHT;
  default:
    LOGERR("Error, received unsupported RobotMoveType: %hhu", moveType);
    return MoveType::FORWARD;
  }
}

//TODO create a separate topic constants header file
constexpr auto ROBOT_MOVE_TYPE_TOPIC = "moveType";
constexpr auto ENABLE_ROBOT_INPUT_TOPIC = "enableInput";
}

CollectorControllerExternalBridge::CollectorControllerExternalBridge()
    : Node("CollectorControllerExternalBridge") {

}

int32_t CollectorControllerExternalBridge::init(
    const CollectorControllerExternalBridgeOutInterface &interface) {
  _outInterface = interface;
  if (nullptr == _outInterface.invokeActionEventCb) {
    LOGERR("Error, nullptr provided for InvokeActionEventCb");
    return FAILURE;
  }
  if (nullptr == _outInterface.moveButtonClickCb) {
    LOGERR("Error, nullptr provided for MoveButtonClickCb");
    return FAILURE;
  }

  using namespace std::placeholders;
  constexpr auto queueSize = 10;
  _playerActSubscriber = create_subscription<RobotMoveType>(
      ROBOT_MOVE_TYPE_TOPIC, queueSize,
      std::bind(&CollectorControllerExternalBridge::onMoveMsg, this, _1));

  _getCoinsService = create_service<GetCurrentCoins>("getCurrentCoins",
      std::bind(&CollectorControllerExternalBridge::handleService, this, _1,
          _2));

  _playerEnableInputPublisher = create_publisher<Empty>(
      ENABLE_ROBOT_INPUT_TOPIC, queueSize);

  return SUCCESS;
}

void CollectorControllerExternalBridge::publishEnablePlayerInput() {
  _playerEnableInputPublisher->publish(Empty());
}

void CollectorControllerExternalBridge::onMoveMsg(
    const RobotMoveType::SharedPtr msg) {
  const auto moveType = getMoveType(msg->move_type);
  const auto f = [this, moveType]() {
    _outInterface.moveButtonClickCb(moveType);
  };

  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

void CollectorControllerExternalBridge::handleService(
    [[maybe_unused]]const std::shared_ptr<GetCurrentCoins::Request> request,
    std::shared_ptr<GetCurrentCoins::Response> response) {
  const auto f = [this, &response]() {
    response->coins = _coins++;
  };

  _outInterface.invokeActionEventCb(f, ActionEventType::BLOCKING);
}

