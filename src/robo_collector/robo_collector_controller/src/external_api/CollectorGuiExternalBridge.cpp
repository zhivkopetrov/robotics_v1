//Corresponding header
#include "robo_collector_controller/external_api/CollectorGuiExternalBridge.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/data_type/EnumClassUtils.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers


  using robo_collector_interfaces::msg::RobotMoveType;

namespace {
//TODO create separate message helper utility file
int8_t getMoveTypeField(MoveType moveType) {
  switch (moveType) {
  case MoveType::FORWARD:
    return RobotMoveType::FORWARD;
  case MoveType::ROTATE_LEFT:
    return RobotMoveType::ROTATE_LEFT;
  case MoveType::ROTATE_RIGHT:
    return RobotMoveType::ROTATE_RIGHT;
  default:
    LOGERR("Error, received unsupported MoveType: %d", getEnumValue(moveType));
    return RobotMoveType::FORWARD;
  }
}

//TODO create a separate topic contants header file
constexpr auto ROBOT_MOVE_TYPE_TOPIC = "moveType";
}

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
  constexpr auto QoS = 10;
  _robotActPublisher =
      create_publisher<RobotMoveType>(ROBOT_MOVE_TYPE_TOPIC, QoS);

  return SUCCESS;
}

void CollectorGuiExternalBridge::publishToggleSettings() const {

}

void CollectorGuiExternalBridge::publishToggleHelp() const {

}

void CollectorGuiExternalBridge::publishRobotAct(MoveType moveType) const {
  LOGC("Inside publishRobotAct");
  RobotMoveType msg;
  msg.move_type = getMoveTypeField(moveType);
  LOGC("about to publish: %hhu", msg.move_type);
  _robotActPublisher->publish(msg);
  LOGC("publish successful");
}


