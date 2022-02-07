//Corresponding header
#include "robo_collector_gui/external_api/ControllerExternalBridge.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

using robo_collector_interfaces::msg::RobotMoveType;
using robo_collector_interfaces::srv::GetCurrentCoins;

namespace {
//TODO create separate message helper utility file
MoveType getMoveType(const RobotMoveType::SharedPtr &msg) {
  switch (msg->move_type) {
  case RobotMoveType::FORWARD:
    return MoveType::FORWARD;
  case RobotMoveType::ROTATE_LEFT:
    return MoveType::ROTATE_LEFT;
  case RobotMoveType::ROTATE_RIGHT:
    return MoveType::ROTATE_RIGHT;
  default:
    LOGERR("Error, received unsupported RobotMoveType: %hhu", msg->move_type);
    return MoveType::FORWARD;
  }
}

//TODO create a separate topic contants header file
constexpr auto ROBOT_MOVE_TYPE_TOPIC = "moveType";
}

ControllerExternalBridge::ControllerExternalBridge()
    : Node("ControllerExternalBridge") {

}

int32_t ControllerExternalBridge::init(
    const ControllerExternalBridgeOutInterface &interface) {
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
  constexpr auto QoS = 10;
  _playerDirSubscriber = create_subscription<RobotMoveType>(
      ROBOT_MOVE_TYPE_TOPIC, QoS,
      std::bind(&ControllerExternalBridge::onMoveMsg, this, _1));

  _getCoinsService = create_service<GetCurrentCoins>("getCurrentCoins",
      std::bind(&ControllerExternalBridge::handleService, this, _1, _2));

  return SUCCESS;
}

void ControllerExternalBridge::onMoveMsg(const RobotMoveType::SharedPtr msg) {
  const auto moveType = getMoveType(msg);
  const auto f = [this, moveType]() {
    _outInterface.moveButtonClickCb(moveType);
  };

  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

void ControllerExternalBridge::handleService(
    [[maybe_unused]]const std::shared_ptr<GetCurrentCoins::Request> request,
    std::shared_ptr<GetCurrentCoins::Response> response) {
  const auto f = [this, &response](){
    response->coins = _coins++;
  };

  _outInterface.invokeActionEventCb(f, ActionEventType::BLOCKING);
}

