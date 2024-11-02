//Corresponding header
#include "ur_control_common/external_api/UrControlCommonExternalBridge.h"

//System headers

//Other libraries headers
#include "urscript_common/defines/UrScriptTopics.h"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/log/Log.h"

//Own components headers
#include "ur_control_common/defines/UrControlCommonTopics.h"
#include "ur_control_common/external_api/config/UrContolCommonExternalBridgeConfig.h"

namespace {

using namespace std::literals;

template <typename T>
void waitForService(T &client) {
  const char *serviceName = client->get_service_name();
  while (!client->wait_for_service(1s)) {
    LOG("Service [%s] not available. Waiting 1s ...", serviceName);
  }
}

} //end anonymous namespace

UrControlCommonExternalBridge::UrControlCommonExternalBridge(
  const std::string& nodeName) : Node(nodeName) {

}

ErrorCode UrControlCommonExternalBridge::init(
    [[maybe_unused]]const UrContolCommonExternalBridgeConfig &cfg,
    const UrControlCommonExternalBridgeOutInterface &interface) {
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

void UrControlCommonExternalBridge::publishURScript(
    const UrScriptPayload &data) const {
  String msg;
  msg.data = data;
  _urscriptPublisher->publish(msg);
}

void UrControlCommonExternalBridge::invokeURScriptService(
    const UrScriptPayload &data) const {
  auto request = std::make_shared<UrScript::Request>();
  request->data = data;
  auto result = _urscriptService->async_send_request(request);
  std::shared_ptr<UrScript::Response> response = result.get();

  if (!response->success) {
    LOGERR("Service call to [%s] failed with error_reason: [%s]",
        _urscriptService->get_service_name(),
        response->error_reason.c_str());
    return;
  }
}

void UrControlCommonExternalBridge::invokeURScriptPreemptService() const {
  auto request = std::make_shared<Trigger::Request>();
  auto result = _urscriptPreemptService->async_send_request(request);
  std::shared_ptr<Trigger::Response> response = result.get();

  if (!response->success) {
    LOGERR("Service call to [%s] failed with error_reason: [%s]",
        _urscriptService->get_service_name(),
        response->message.c_str());
    return;
  }
}

ErrorCode UrControlCommonExternalBridge::initOutInterface(
    const UrControlCommonExternalBridgeOutInterface &outInterface) {
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

ErrorCode UrControlCommonExternalBridge::initCommunication() {
  using namespace std::placeholders;

  constexpr auto queueSize = 10;
  const rclcpp::QoS qos(queueSize);

  rclcpp::SubscriptionOptions subsriptionOptions;
  subsriptionOptions.callback_group = _subscriberCallbackGroup;

  rclcpp::PublisherOptions publisherOptions;
  publisherOptions.callback_group = _publishersCallbackGroup;

  _urscriptPublisher = create_publisher<String>(URSCRIPT_TOPIC, qos,
      publisherOptions);

  _urscriptService = create_client<UrScript>(URSCRIPT_SERVICE,
      rmw_qos_profile_services_default, _publishersCallbackGroup);

  _urscriptPreemptService = create_client<Trigger>(URSCRIPT_SERVICE_PREEMPT,
      rmw_qos_profile_services_default, _publishersCallbackGroup);

  _robotModeSubscriber = create_subscription<RobotModeType>(ROBOT_MODE_TOPIC,
      qos, std::bind(&UrControlCommonExternalBridge::onRobotModeMsg, this, _1),
      subsriptionOptions);

  _safetyModeSubscriber = create_subscription<SafetyModeType>(SAFETY_MODE_TOPIC,
      qos, std::bind(&UrControlCommonExternalBridge::onSafetyModeMsg, this, _1),
      subsriptionOptions);

  waitForService(_urscriptService);
  waitForService(_urscriptPreemptService);

  return ErrorCode::SUCCESS;
}

void UrControlCommonExternalBridge::onRobotModeMsg(
    const RobotModeType::SharedPtr msg) {
  const RobotMode mode = toEnum<RobotMode>(msg->mode);
  const auto f = [this, mode]() {
    _outInterface.robotModeChangeCb(mode);
  };
  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

void UrControlCommonExternalBridge::onSafetyModeMsg(
    const SafetyModeType::SharedPtr msg) {
  const SafetyMode mode = toEnum<SafetyMode>(msg->mode);
  const auto f = [this, mode]() {
    _outInterface.safetyModeChangeCb(mode);
  };
  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}
