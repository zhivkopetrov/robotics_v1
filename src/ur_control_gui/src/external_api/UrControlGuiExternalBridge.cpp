//Corresponding header
#include "ur_control_gui/external_api/UrControlGuiExternalBridge.h"

//System headers

//Other libraries headers
#include "utils/data_type/EnumClassUtils.h"
#include "utils/Log.h"

//Own components headers
#include "ur_control_gui/defines/UrControlGuiTopics.h"

namespace {
constexpr auto NODE_NAME = "UrControlGuiExternalBridge";
}

UrControlGuiExternalBridge::UrControlGuiExternalBridge() : Node(NODE_NAME) {

}

ErrorCode UrControlGuiExternalBridge::init(
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

