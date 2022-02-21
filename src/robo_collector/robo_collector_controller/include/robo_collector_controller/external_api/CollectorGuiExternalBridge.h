#ifndef ROBO_COLLECTOR_CONTROLLER_COLLECTORGUIEXTERNALBRIDGE_H_
#define ROBO_COLLECTOR_CONTROLLER_COLLECTORGUIEXTERNALBRIDGE_H_

//C system headers

//C++ system headers

//Other libraries headers
#include <rclcpp/node.hpp>
#include "std_msgs/msg/empty.hpp"
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

  void publishToggleSettings();
  void publishToggleHelp();
  void publishRobotAct(MoveType moveType);

private:
  typedef robo_collector_interfaces::msg::RobotMoveType RobotMoveType;
  typedef std_msgs::msg::Empty Empty;

  void onEnableRobotTurnMsg(const Empty::SharedPtr msg);

  CollectorGuiExternalBridgeOutInterface _outInterface;
  rclcpp::Publisher<RobotMoveType>::SharedPtr _robotActPublisher;

  rclcpp::Subscription<Empty>::SharedPtr _enableRobotTurn;
};

#endif /* ROBO_COLLECTOR_CONTROLLER_COLLECTORGUIEXTERNALBRIDGE_H_ */
