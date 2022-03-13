//Corresponding header
#include "robo_cleaner_gui/external_api/CleanerControllerExternalBridge.h"

//System headers

//Other libraries headers
#include "robo_common/defines/RoboCommonDefines.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

using robo_cleaner_interfaces::msg::RobotMoveType;
using robo_cleaner_interfaces::srv::RobotMove;

namespace {
//TODO create separate message helper utility file
ErrorCode getMoveType(int8_t moveType, MoveType& outMoveType) {
  switch (moveType) {
  case RobotMoveType::FORWARD:
    outMoveType = MoveType::FORWARD;
    break;
  case RobotMoveType::ROTATE_LEFT:
    outMoveType = MoveType::ROTATE_LEFT;
    break;
  case RobotMoveType::ROTATE_RIGHT:
    outMoveType = MoveType::ROTATE_RIGHT;
    break;
  default:
    LOGERR("Error, received unsupported RobotMoveType: %hhu", moveType);
    return ErrorCode::FAILURE;
  }
  return ErrorCode::SUCCESS;
}

//TODO create a separate topic contants header file
constexpr auto ROBOT_MOVE_TYPE_TOPIC = "moveType";
}

CleanerControllerExternalBridge::CleanerControllerExternalBridge()
    : Node("CleanerControllerExternalBridge") {

}

ErrorCode CleanerControllerExternalBridge::init(
    const CleanerControllerExternalBridgeOutInterface &interface) {
  _outInterface = interface;
  if (nullptr == _outInterface.invokeActionEventCb) {
    LOGERR("Error, nullptr provided for InvokeActionEventCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.robotActCb) {
    LOGERR("Error, nullptr provided for RobotActCb");
    return ErrorCode::FAILURE;
  }

  using namespace std::placeholders;
  constexpr auto QoS = 10;
  _playerDirSubscriber = create_subscription<RobotMoveType>(
      ROBOT_MOVE_TYPE_TOPIC, QoS,
      std::bind(&CleanerControllerExternalBridge::onMoveMsg, this, _1));

  _robotMoveService = create_service<RobotMove>("robotMove",
      std::bind(&CleanerControllerExternalBridge::handleService, this, _1, _2));

  return ErrorCode::SUCCESS;
}

void CleanerControllerExternalBridge::publishShutdownController() {
  //TODO publish on a topic

  const auto f = [this]() {
    _outInterface.systemShutdownCb();
  };
  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

void CleanerControllerExternalBridge::onMoveMsg(
    const RobotMoveType::SharedPtr msg) {
  MoveType moveType;
  const auto err = getMoveType(msg->move_type, moveType);
  if (ErrorCode::FAILURE == err) {
    return;
  }

  const auto f = [moveType]() {
//    _outInterface.moveButtonClickCb(moveType);
    //TODO remove me after test
    if (MoveType::FORWARD == moveType) {
    }
  };

  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

void CleanerControllerExternalBridge::handleService(
    const std::shared_ptr<robo_cleaner_interfaces::srv::RobotMove::Request> request,
    std::shared_ptr<robo_cleaner_interfaces::srv::RobotMove::Response> response) {
  MoveType moveType;
  const auto err = getMoveType(request->robot_move_type.move_type, moveType);
  if (ErrorCode::FAILURE == err) {
    response->success = false;
    return;
  }

  const auto f = [this, moveType, &response]() {
    _outInterface.robotActCb(moveType);
    response->success = true;
  };

  _outInterface.invokeActionEventCb(f, ActionEventType::BLOCKING);
}

