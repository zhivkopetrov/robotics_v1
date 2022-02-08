#ifndef ROBO_COLLECTOR_GUI_CONTROLLEREXTERNALBRIDGE_H_
#define ROBO_COLLECTOR_GUI_CONTROLLEREXTERNALBRIDGE_H_

//C system headers

//C++ system headers

//Other libraries headers
#include <rclcpp/node.hpp>
#include "robo_collector_interfaces/msg/robot_move_type.hpp"
#include "robo_collector_interfaces/srv/get_current_coins.hpp"
#include "game_engine/defines/ActionEventDefines.h"

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiFunctionalDefines.h"

//Forward declarations

struct ControllerExternalBridgeOutInterface {
  InvokeActionEventCb invokeActionEventCb;
  MoveButtonClickCb moveButtonClickCb;
};

class ControllerExternalBridge: public rclcpp::Node {
public:
  ControllerExternalBridge();

  int32_t init(const ControllerExternalBridgeOutInterface &interface);

private:
  void onMoveMsg(
      const robo_collector_interfaces::msg::RobotMoveType::SharedPtr msg);

  //TODO remove after test
  void handleService(
      const std::shared_ptr<
          robo_collector_interfaces::srv::GetCurrentCoins::Request> request,
      std::shared_ptr<
          robo_collector_interfaces::srv::GetCurrentCoins::Response> response);

  ControllerExternalBridgeOutInterface _outInterface;

  rclcpp::Subscription<robo_collector_interfaces::msg::RobotMoveType>::SharedPtr
    _playerDirSubscriber;

  //TODO remove after test
  rclcpp::Service<robo_collector_interfaces::srv::GetCurrentCoins>::SharedPtr
    _getCoinsService;

  //TODO remove after test
  int64_t _coins = 0;
};

#endif /* ROBO_COLLECTOR_GUI_CONTROLLEREXTERNALBRIDGE_H_ */
