//Corresponding header
#include "ur_control_common/external_api/UrControlCommonExternalBridge.h"

//System headers

//Other libraries headers
#include "urscript_common/defines/UrScriptTopics.h"
#include "urscript_common/defines/RobotDefines.h"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/Log.h"

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

UrControlCommonExternalBridge::~UrControlCommonExternalBridge() noexcept {
  publishDeleteAllMarkers();
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

void UrControlCommonExternalBridge::publishMarker() {
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = 0;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 0;

  std_msgs::msg::ColorRGBA colour;
  colour.a = 0.5;
  colour.r = 1;
  colour.g = 0;
  colour.b = 0;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = ur_links::TOOL0_NAME;
  marker.header.stamp = get_clock()->now();

  marker.id           = Marker::ADD;
  marker.type         = Marker::CUBE;
  marker.scale.x      = 0.25;
  marker.scale.y      = 0.25;
  marker.scale.z      = 0.25;
  marker.color        = colour;
  marker.pose         = pose;
  marker.frame_locked = true;

  _markerPublisher->publish(marker);
}

void UrControlCommonExternalBridge::publishDeleteAllMarkers() {
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = get_clock()->now();
  marker.id     = Marker::DELETEALL;
  marker.action = Marker::DELETEALL;

  _markerPublisher->publish(marker);
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

  _markerPublisher = create_publisher<Marker>("bazinga", qos,
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
