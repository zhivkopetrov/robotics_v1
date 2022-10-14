//Corresponding header
#include "robo_collector_gui/external_api/CollectorControllerExternalBridge.h"

//System headers

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

ErrorCode CollectorControllerExternalBridge::init(
    const CollectorControllerExternalBridgeOutInterface &interface) {
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

void CollectorControllerExternalBridge::publishEnablePlayerInput() {
  const auto f = [this]() {
    if (ControllerStatus::SHUTTING_DOWN == _controllerStatus) {
      return;
    }
    _controllerStatus = ControllerStatus::IDLE;
  };
  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);

  _playerEnableInputPublisher->publish(Empty());
}

//called from the main thread
void CollectorControllerExternalBridge::publishShutdownController() {
  _controllerStatus = ControllerStatus::SHUTTING_DOWN;

  _shutdownControllerPublisher->publish(Empty());
}

ErrorCode CollectorControllerExternalBridge::initOutInterface(
    const CollectorControllerExternalBridgeOutInterface &outInterface) {
  _outInterface = outInterface;
  if (nullptr == _outInterface.invokeActionEventCb) {
    LOGERR("Error, nullptr provided for InvokeActionEventCb");
    return ErrorCode::FAILURE;
  }
  if (nullptr == _outInterface.moveButtonClickCb) {
    LOGERR("Error, nullptr provided for MoveButtonClickCb");
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

  return ErrorCode::SUCCESS;
}

ErrorCode CollectorControllerExternalBridge::initCommunication() {
  using namespace std::placeholders;
  constexpr size_t queueSize = 10;

  const rclcpp::QoS qos(queueSize);
  rclcpp::QoS latchQoS = qos;

  //enable message latching for late joining subscribers
  latchQoS.transient_local();

  rclcpp::SubscriptionOptions subsriptionOptions;
  subsriptionOptions.callback_group = _subscriberCallbackGroup;

  rclcpp::PublisherOptions publisherOptions;
  publisherOptions.callback_group = _publishersCallbackGroup;

  //only _userAuthenticateSubscriber should use the 'latchQoS' object
  //this will allow independent start order of the
  //client(robo_collector_controler) and the server(robo_collector_gui)
  _userAuthenticateSubscriber = create_subscription<UserAuthenticate>(
      USER_AUTHENTICATE_TOPIC, latchQoS,
      std::bind(&CollectorControllerExternalBridge::onUserAuthenticateMsg, this,
          _1), subsriptionOptions);

  _playerActSubscriber = create_subscription<RobotMoveType>(
      ROBOT_MOVE_TYPE_TOPIC, qos,
      std::bind(&CollectorControllerExternalBridge::onMoveMsg, this, _1),
      subsriptionOptions);

  _toggleHelpPageSubscriber = create_subscription<Empty>(TOGGLE_HELP_PAGE_TOPIC,
      qos,
      std::bind(&CollectorControllerExternalBridge::onToggleHelpPageMsg, this,
          _1), subsriptionOptions);

  _toggleDebugInfoSubscriber = create_subscription<Empty>(
      TOGGLE_DEBUG_INFO_TOPIC, qos,
      std::bind(&CollectorControllerExternalBridge::onToggleDebugInfoMsg, this,
          _1), subsriptionOptions);

  _setDebugMsgSubscriber = create_subscription<String>(DEBUG_MSG_TOPIC, qos,
      std::bind(&CollectorControllerExternalBridge::onDebugMsg, this, _1),
      subsriptionOptions);

  _playerEnableInputPublisher = create_publisher<Empty>(
      ENABLE_ROBOT_INPUT_TOPIC, qos, publisherOptions);

  _shutdownControllerPublisher = create_publisher<Empty>(
      SHUTDOWN_CONTROLLER_TOPIC, qos, publisherOptions);

  return ErrorCode::SUCCESS;
}

void CollectorControllerExternalBridge::onUserAuthenticateMsg(
    const UserAuthenticate::SharedPtr msg) {
  const UserData data = { .user = msg->user, .repository = msg->repository,
      .commitSha = msg->commit_sha };

  const auto f = [this, data]() {
    _outInterface.setUserDataCb(data);
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

  bool success = true;
  const auto f = [this, &success, moveType]() {
    if (ControllerStatus::ACTIVE == _controllerStatus) {
      success = false;
      LOGERR("Rejecting Move Request with type: [%d], because another one is "
             "already active", getEnumValue(moveType));
      return;
    }
    if (ControllerStatus::SHUTTING_DOWN == _controllerStatus) {
      success = false;
      LOGERR("Rejecting Move Request with type: [%d], because controller is "
             "shutting down", getEnumValue(moveType));
      return;
    }

    _controllerStatus = ControllerStatus::ACTIVE;
  };
  _outInterface.invokeActionEventCb(f, ActionEventType::BLOCKING);
  if (!success) {
    return;
  }

  const auto f2 = [this, moveType]() {
    _outInterface.moveButtonClickCb(moveType);
  };

  _outInterface.invokeActionEventCb(f2, ActionEventType::NON_BLOCKING);
}

void CollectorControllerExternalBridge::onToggleHelpPageMsg(
    [[maybe_unused]]const Empty::SharedPtr msg) {
  const auto f = [this]() {
    _outInterface.toggleHelpPageCb();
  };

  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

void CollectorControllerExternalBridge::onToggleDebugInfoMsg(
    [[maybe_unused]]const Empty::SharedPtr msg) {
  const auto f = [this]() {
    _outInterface.toggleDebugInfoCb();
  };

  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

void CollectorControllerExternalBridge::onDebugMsg(
    const String::SharedPtr msg) {
  const auto f = [this, msg]() {
    _outInterface.setDebugMsgCb(msg->data);
  };

  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

