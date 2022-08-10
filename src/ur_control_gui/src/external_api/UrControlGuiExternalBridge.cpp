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
    const UrContolGuiExternalBridgeConfig &cfg,
    const UrControlGuiExternalBridgeOutInterface &interface) {

  constexpr auto tool = "netcat";
  constexpr auto closeConnectionCommand = "-q 0";
  constexpr auto toolCommand = "<<<";
  _scriptPrefix = tool;
  _scriptPrefix.append(" ").append(cfg.robotIp).append(" ").append(
      std::to_string(cfg.robotInterfacePort)).append(" ").append(
      closeConnectionCommand).append(" ").append(toolCommand).append(" ");

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

//TODO remove me
#include <cstdlib>

void UrControlGuiExternalBridge::publishURScript(
    const std::string &data) const {
  String msg;
  msg.data = data;
  _urscriptPublisher->publish(msg);

  //TODO remove me
  const std::string systemString = _scriptPrefix + "\"" + data + "\"";
  if (-1 == std::system(msg.data.c_str())) {
    LOGERR("std::system() failed");
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

  _urscriptPublisher = create_publisher<String>(URSCRIPT_TOPIC, qos);

  _robotModeSubscriber = create_subscription<RobotModeType>(ROBOT_MODE_TOPIC,
      qos, std::bind(&UrControlGuiExternalBridge::onRobotModeMsg, this, _1));

  _safetyModeSubscriber = create_subscription<SafetyModeType>(SAFETY_MODE_TOPIC,
      qos, std::bind(&UrControlGuiExternalBridge::onSafetyModeMsg, this, _1));

  return ErrorCode::SUCCESS;
}

void UrControlGuiExternalBridge::onRobotModeMsg(
    const RobotModeType::SharedPtr msg) {
  const RobotMode mode = toEnum<RobotMode>(msg->mode);
  const auto f = [this, mode](){
    _outInterface.robotModeChangeCb(mode);
  };
  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

void UrControlGuiExternalBridge::onSafetyModeMsg(
    const SafetyModeType::SharedPtr msg) {
  const SafetyMode mode = toEnum<SafetyMode>(msg->mode);
  const auto f = [this, mode](){
    _outInterface.safetyModeChangeCb(mode);
  };
  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

