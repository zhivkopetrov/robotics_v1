#ifndef ROBO_MINER_GUI_MINERCONTROLLEREXTERNALBRIDGE_H_
#define ROBO_MINER_GUI_MINERCONTROLLEREXTERNALBRIDGE_H_

//C system headers

//C++ system headers

//Other libraries headers
#include <rclcpp/node.hpp>
#include <std_msgs/msg/empty.hpp>
#include "robo_miner_interfaces/srv/robot_move.hpp"
#include "robo_miner_interfaces/srv/field_map_check.hpp"
#include "game_engine/defines/ActionEventDefines.h"
#include "robo_common/defines/RoboCommonFunctionalDefines.h"

//Own components headers

//Forward declarations
class MovementWatcher;
class SolutionValidator;

struct MinerControllerExternalBridgeOutInterface {
  InvokeActionEventCb invokeActionEventCb;
  RobotActCb robotActCb;
  SystemShutdownCb systemShutdownCb;
  MovementWatcher *movementWatcher = nullptr;
  SolutionValidator *solutionValidator = nullptr;
};

class MinerControllerExternalBridge: public rclcpp::Node {
public:
  MinerControllerExternalBridge();

  int32_t init(const MinerControllerExternalBridgeOutInterface &interface);

  void publishShutdownController();

private:
  typedef std_msgs::msg::Empty Empty;
  typedef robo_miner_interfaces::srv::RobotMove RobotMove;
  typedef robo_miner_interfaces::srv::FieldMapCheck FieldMapCheck;

  void handleRobotMoveService(const std::shared_ptr<RobotMove::Request> request,
                              std::shared_ptr<RobotMove::Response> response);

  void handleFieldMapCheckService(
      const std::shared_ptr<FieldMapCheck::Request> request,
      std::shared_ptr<FieldMapCheck::Response> response);

  MinerControllerExternalBridgeOutInterface _outInterface;

  rclcpp::Service<RobotMove>::SharedPtr _robotMoveService;
  rclcpp::Service<FieldMapCheck>::SharedPtr _fieldMapCheckService;
  rclcpp::Publisher<Empty>::SharedPtr _shutdownControllerPublisher;
};

#endif /* ROBO_MINER_GUI_MINERCONTROLLEREXTERNALBRIDGE_H_ */
