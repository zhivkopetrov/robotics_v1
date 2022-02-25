#ifndef ROBO_MINER_GUI_MINERCONTROLLEREXTERNALBRIDGE_H_
#define ROBO_MINER_GUI_MINERCONTROLLEREXTERNALBRIDGE_H_

//C system headers

//C++ system headers

//Other libraries headers
#include <rclcpp/node.hpp>
#include <std_msgs/msg/empty.hpp>
#include "robo_miner_interfaces/srv/robot_move.hpp"
#include "game_engine/defines/ActionEventDefines.h"
#include "robo_common/defines/RoboCommonFunctionalDefines.h"

//Own components headers

//Forward declarations
class MovementWatcher;

struct MinerControllerExternalBridgeOutInterface {
  InvokeActionEventCb invokeActionEventCb;
  RobotActCb robotActCb;
  SystemShutdownCb systemShutdownCb;
  MovementWatcher* movementWatcher = nullptr;
};

class MinerControllerExternalBridge: public rclcpp::Node {
public:
  MinerControllerExternalBridge();

  int32_t init(const MinerControllerExternalBridgeOutInterface &interface);

  void publishShutdownController();

private:
  typedef std_msgs::msg::Empty Empty;
  typedef robo_miner_interfaces::srv::RobotMove RobotMove;

  void handleRobotMoveService(const std::shared_ptr<RobotMove::Request> request,
                              std::shared_ptr<RobotMove::Response> response);

  MinerControllerExternalBridgeOutInterface _outInterface;

  rclcpp::Service<RobotMove>::SharedPtr _robotMoveService;
  rclcpp::Publisher<Empty>::SharedPtr _shutdownControllerPublisher;
};

#endif /* ROBO_MINER_GUI_MINERCONTROLLEREXTERNALBRIDGE_H_ */
