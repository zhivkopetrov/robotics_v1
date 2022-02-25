//Corresponding header
#include "robo_collector_gui/external_api/CollectorControllerExternalBridge.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "robo_collector_common/defines/RoboCollectorTopics.h"
#include "robo_collector_common/message_helpers/RoboCollectorMessageHelpers.h"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

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
  if (nullptr == _outInterface.systemShutdownCb) {
    LOGERR("Error, nullptr provided for SystemShutdownCb");
    return FAILURE;
  }

  using namespace std::placeholders;
  constexpr auto queueSize = 10;
  _playerActSubscriber = create_subscription<RobotMoveType>(
      ROBOT_MOVE_TYPE_TOPIC, queueSize,
      std::bind(&CollectorControllerExternalBridge::onMoveMsg, this, _1));

  _playerEnableInputPublisher = create_publisher<Empty>(
      ENABLE_ROBOT_INPUT_TOPIC, queueSize);

  _shutdownControllerPublisher = create_publisher<Empty>(
      SHUTDOWN_CONTROLLER_TOPIC, queueSize);

  return SUCCESS;
}

void CollectorControllerExternalBridge::publishEnablePlayerInput() {
  _playerEnableInputPublisher->publish(Empty());
}

void CollectorControllerExternalBridge::publishShutdownController() {
  _shutdownControllerPublisher->publish(Empty());

  const auto f = [this]() {
    _outInterface.systemShutdownCb();
  };
  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

void CollectorControllerExternalBridge::onMoveMsg(
    const RobotMoveType::SharedPtr msg) {
  const auto moveType = getMoveType(msg->move_type);
  if (MoveType::UNKNOWN == moveType) {
    LOGERR("Error, received unsupported MoveType: %d", getEnumValue(moveType));
    return;
  }

  const auto f = [this, moveType]() {
    _outInterface.moveButtonClickCb(moveType);
  };

  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}


