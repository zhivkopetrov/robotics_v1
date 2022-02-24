//Corresponding header
#include "robo_collector_controller/external_api/CollectorGuiExternalBridge.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "robo_collector_common/defines/RoboCollectorTopics.h"
#include "robo_collector_common/message_helpers/RoboCollectorMessageHelpers.h"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

CollectorGuiExternalBridge::CollectorGuiExternalBridge()
    : Node("CollectorGuiExternalBridge") {

}

int32_t CollectorGuiExternalBridge::init(
    const CollectorGuiExternalBridgeOutInterface &interface) {
  _outInterface = interface;
  if (nullptr == _outInterface.invokeActionEventCb) {
    LOGERR("Error, nullptr provided for InvokeActionEventCb");
    return FAILURE;
  }

  if (nullptr == _outInterface.enablePlayerInputCb) {
    LOGERR("Error, nullptr provided for EnablePlayerInputCb");
    return FAILURE;
  }

  using namespace std::placeholders;
  constexpr auto queueSize = 10;
  _robotActPublisher = create_publisher<RobotMoveType>(ROBOT_MOVE_TYPE_TOPIC,
      queueSize);

  _enableRobotTurnSubscription = create_subscription<Empty>(
      ENABLE_ROBOT_INPUT_TOPIC, queueSize,
      std::bind(&CollectorGuiExternalBridge::onEnableRobotTurnMsg, this, _1));

  _shutdownControllerSubscription = create_subscription<Empty>(
      SHUTDOWN_CONTROLLER_TOPIC, queueSize,
      std::bind(&CollectorGuiExternalBridge::onControllerShutdownMsg, this,
          _1));

  return SUCCESS;
}

void CollectorGuiExternalBridge::publishToggleSettings() {

}

void CollectorGuiExternalBridge::publishToggleHelp() {

}

void CollectorGuiExternalBridge::publishRobotAct(MoveType moveType) {
  RobotMoveType msg;
  msg.move_type = getMoveTypeField(moveType);
  _robotActPublisher->publish(msg);
}

void CollectorGuiExternalBridge::onEnableRobotTurnMsg(
    [[maybe_unused]]const Empty::SharedPtr msg) {
  const auto f = [this]() {
    _outInterface.enablePlayerInputCb();
  };

  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

void CollectorGuiExternalBridge::onControllerShutdownMsg(
    [[maybe_unused]]const Empty::SharedPtr msg) {
  const auto f = [this]() {
    _outInterface.systemShutdownCb();
  };

  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

