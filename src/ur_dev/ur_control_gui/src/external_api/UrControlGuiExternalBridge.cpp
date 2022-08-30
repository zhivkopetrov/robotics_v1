//Corresponding header
#include "ur_control_gui/external_api/UrControlGuiExternalBridge.h"

//System headers

//Other libraries headers
#include "utils/data_type/EnumClassUtils.h"
#include "utils/Log.h"

//Own components headers
#include "ur_control_gui/defines/UrControlGuiTopics.h"
#include "ur_control_gui/external_api/config/UrContolGuiExternalBridgeConfig.h"

namespace {
constexpr auto NODE_NAME = "UrControlGuiExternalBridge";
}

UrControlGuiExternalBridge::UrControlGuiExternalBridge()
    : Node(NODE_NAME) {

}

ErrorCode UrControlGuiExternalBridge::init(
    [[maybe_unused]]const UrContolGuiExternalBridgeConfig &cfg,
    const UrControlGuiExternalBridgeOutInterface &interface) {
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

void UrControlGuiExternalBridge::publishURScript(
    const std::string &data) const {
  String msg;
  msg.data = data;
  _urscriptPublisher->publish(msg);
}

void UrControlGuiExternalBridge::invokeURScriptService(
    const std::string &data) const {
  auto request = std::make_shared<UrScript::Request>();
  request->data = data;
  auto result = _urscriptPublisherService->async_send_request(request);
  std::shared_ptr<UrScript::Response> response = result.get();

  if (!response->ok) {
    LOGERR("Service call to [%s] failed",
        _urscriptPublisherService->get_service_name());
    return;
  }
}

ErrorCode UrControlGuiExternalBridge::initOutInterface(
    const UrControlGuiExternalBridgeOutInterface &outInterface) {
  _outInterface = outInterface;
  if (nullptr == _outInterface.invokeActionEventCb) {
    LOGERR("Error, nullptr provided for InvokeActionEventCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.robotModeChangeCb) {
    LOGERR("Error, nullptr provided for RobotModeChangeCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.safetyModeChangeCb) {
    LOGERR("Error, nullptr provided for SafetyModeChangeCb");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode UrControlGuiExternalBridge::initCommunication() {
  using namespace std::placeholders;

  constexpr auto queueSize = 10;
  rclcpp::QoS qos(queueSize);

  rclcpp::SubscriptionOptions subsriptionOptions;
  subsriptionOptions.callback_group = _subscriberCallbackGroup;

  rclcpp::PublisherOptions publisherOptions;
  publisherOptions.callback_group = _publishersCallbackGroup;

  _urscriptPublisher = create_publisher<String>(URSCRIPT_TOPIC, qos,
      publisherOptions);

  _urscriptPublisherService = create_client<UrScript>(URSCRIPT_SERVICE,
      rmw_qos_profile_services_default, _publishersCallbackGroup);

  _robotModeSubscriber = create_subscription<RobotModeType>(ROBOT_MODE_TOPIC,
      qos, std::bind(&UrControlGuiExternalBridge::onRobotModeMsg, this, _1),
      subsriptionOptions);

  _safetyModeSubscriber = create_subscription<SafetyModeType>(SAFETY_MODE_TOPIC,
      qos, std::bind(&UrControlGuiExternalBridge::onSafetyModeMsg, this, _1),
      subsriptionOptions);

  return ErrorCode::SUCCESS;
}

void UrControlGuiExternalBridge::onRobotModeMsg(
    const RobotModeType::SharedPtr msg) {
  const RobotMode mode = toEnum<RobotMode>(msg->mode);
  const auto f = [this, mode]() {
    _outInterface.robotModeChangeCb(mode);
  };
  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

void UrControlGuiExternalBridge::onSafetyModeMsg(
    const SafetyModeType::SharedPtr msg) {
  const SafetyMode mode = toEnum<SafetyMode>(msg->mode);
  const auto f = [this, mode]() {
    _outInterface.safetyModeChangeCb(mode);
  };
  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}
