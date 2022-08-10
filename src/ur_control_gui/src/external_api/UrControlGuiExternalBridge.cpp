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

  if (nullptr == _outInterface.systemShutdownCb) {
    LOGERR("Error, nullptr provided for SystemShutdownCb");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode UrControlGuiExternalBridge::initCommunication() {
  constexpr auto queueSize = 10;
  rclcpp::QoS qos(queueSize);

  _urscriptPublisher = create_publisher<String>(URSCRIPT_TOPIC, qos);

  return ErrorCode::SUCCESS;
}

