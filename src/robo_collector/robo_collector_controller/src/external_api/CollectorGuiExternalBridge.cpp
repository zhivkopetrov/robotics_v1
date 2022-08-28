//Corresponding header
#include "robo_collector_controller/external_api/CollectorGuiExternalBridge.h"

//System headers

//Other libraries headers
#include "robo_collector_common/defines/RoboCollectorTopics.h"
#include "robo_collector_common/message_helpers/RoboCollectorMessageHelpers.h"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/Log.h"

//Own components headers

CollectorGuiExternalBridge::CollectorGuiExternalBridge()
    : Node("CollectorGuiExternalBridge") {

}

ErrorCode CollectorGuiExternalBridge::init(
    const CollectorGuiExternalBridgeOutInterface &interface) {
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

void CollectorGuiExternalBridge::publishToggleDebugInfo() const {
  _toggleDebugInfoPublisher->publish(Empty());
}

void CollectorGuiExternalBridge::publishToggleHelpPage() const {
  _toggleHelpPagePublisher->publish(Empty());
}

void CollectorGuiExternalBridge::publishRobotAct(MoveType moveType) const {
  if (MoveType::UNKNOWN == moveType) {
    LOGERR("Error, received unsupported MoveType: %d", getEnumValue(moveType));
    return;
  }

  RobotMoveType msg;
  msg.move_type = getMoveTypeField(moveType);
  _robotActPublisher->publish(msg);
}

void CollectorGuiExternalBridge::publishUserAuthenticate(const UserData &data) {
  UserAuthenticate msg;
  msg.user = data.user;
  msg.commit_sha = data.commitSha;
  msg.repository = data.repository;
  _userAuthenticatePublisher->publish(msg);
}

ErrorCode CollectorGuiExternalBridge::initOutInterface(
    const CollectorGuiExternalBridgeOutInterface &outInterface) {
  _outInterface = outInterface;
  if (nullptr == _outInterface.invokeActionEventCb) {
    LOGERR("Error, nullptr provided for InvokeActionEventCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.enablePlayerInputCb) {
    LOGERR("Error, nullptr provided for EnablePlayerInputCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.systemShutdownCb) {
    LOGERR("Error, nullptr provided for SystemShutdownCb");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode CollectorGuiExternalBridge::initCommunication() {
  using namespace std::placeholders;
  constexpr auto queueSize = 10;
  _userAuthenticatePublisher = create_publisher<UserAuthenticate>(
      USER_AUTHENTICATE_TOPIC, queueSize);

  rclcpp::QoS qos(queueSize);
  qos.transient_local(); //enable message latching for late joining subscribers

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

  _robotActPublisher = create_publisher<RobotMoveType>(ROBOT_MOVE_TYPE_TOPIC,
      qos, publisherOptions);

  _toggleHelpPagePublisher = create_publisher<Empty>(TOGGLE_HELP_PAGE_TOPIC,
      queueSize, publisherOptions);

  _toggleDebugInfoPublisher = create_publisher<Empty>(TOGGLE_DEBUG_INFO_TOPIC,
      queueSize, publisherOptions);

  _enableRobotTurnSubscription = create_subscription<Empty>(
      ENABLE_ROBOT_INPUT_TOPIC, queueSize,
      std::bind(&CollectorGuiExternalBridge::onEnableRobotTurnMsg, this, _1),
      subsriptionOptions);

  _shutdownControllerSubscription = create_subscription<Empty>(
      SHUTDOWN_CONTROLLER_TOPIC, queueSize,
      std::bind(&CollectorGuiExternalBridge::onControllerShutdownMsg, this, _1),
      subsriptionOptions);

  return ErrorCode::SUCCESS;
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

