#ifndef ROBO_COLLECTOR_GUI_COLLECTORCONTROLLEREXTERNALBRIDGE_H_
#define ROBO_COLLECTOR_GUI_COLLECTORCONTROLLEREXTERNALBRIDGE_H_

//C system headers

//C++ system headers

//Other libraries headers
#include <rclcpp/node.hpp>
#include "robo_collector_interfaces/msg/robot_move_type.hpp"
#include "robo_collector_common/defines/RoboCollectorFunctionalDefines.h"
#include "game_engine/defines/ActionEventDefines.h"

//Own components headers

//Forward declarations

struct CollectorGuiExternalBridgeOutInterface {
  InvokeActionEventCb invokeActionEventCb;
  EnablePlayerInputCb enablePlayerInputCb;
};

class CollectorGuiExternalBridge: public rclcpp::Node {
public:
  CollectorGuiExternalBridge();

  int32_t init(const CollectorGuiExternalBridgeOutInterface &interface);

  void publishToggleSettings() const;
  void publishToggleHelp() const;
  void publishRobotAct(MoveType moveType) const;

private:
//  typedef robo_collector_interfaces::msg::RobotMoveType RobotMoveType;

  CollectorGuiExternalBridgeOutInterface _outInterface;

  rclcpp::Publisher<robo_collector_interfaces::msg::RobotMoveType>::SharedPtr _robotActPublisher;
};

#endif /* ROBO_COLLECTOR_GUI_COLLECTORCONTROLLEREXTERNALBRIDGE_H_ */
