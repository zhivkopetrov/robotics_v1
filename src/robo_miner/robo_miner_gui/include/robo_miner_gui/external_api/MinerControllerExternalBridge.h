#ifndef ROBO_MINER_GUI_MINERCONTROLLEREXTERNALBRIDGE_H_
#define ROBO_MINER_GUI_MINERCONTROLLEREXTERNALBRIDGE_H_

//C system headers

//C++ system headers

//Other libraries headers
#include <rclcpp/node.hpp>
#include "game_engine/defines/ActionEventDefines.h"
#include "robo_common/defines/RoboCommonFunctionalDefines.h"
#include "robo_miner_interfaces/msg/robot_move_type.hpp"
#include "robo_miner_interfaces/srv/robot_move.hpp"

//Own components headers

//Forward declarations

struct MinerControllerExternalBridgeOutInterface {
  InvokeActionEventCb invokeActionEventCb;
};

class MinerControllerExternalBridge: public rclcpp::Node {
public:
  MinerControllerExternalBridge();

  int32_t init(const MinerControllerExternalBridgeOutInterface &interface);

private:
  void onMoveMsg(
      const robo_miner_interfaces::msg::RobotMoveType::SharedPtr msg);

  //TODO remove after test
  void handleService(
      const std::shared_ptr<
        robo_miner_interfaces::srv::RobotMove::Request> request,
      std::shared_ptr<
        robo_miner_interfaces::srv::RobotMove::Response> response);

  MinerControllerExternalBridgeOutInterface _outInterface;

  rclcpp::Subscription<robo_miner_interfaces::msg::RobotMoveType>::SharedPtr
    _playerDirSubscriber;

  rclcpp::Service<robo_miner_interfaces::srv::RobotMove>::SharedPtr
    _robotMoveService;

  //TODO remove me after test
  int32_t _dummy = 0;
};

#endif /* ROBO_MINER_GUI_MINERCONTROLLEREXTERNALBRIDGE_H_ */
