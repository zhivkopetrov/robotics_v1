#ifndef ROBO_COLLECTOR_GUI_COLLECTORCONTROLLEREXTERNALBRIDGE_H_
#define ROBO_COLLECTOR_GUI_COLLECTORCONTROLLEREXTERNALBRIDGE_H_

//C system headers

//C++ system headers

//Other libraries headers
#include <rclcpp/node.hpp>
#include "std_msgs/msg/empty.hpp"
#include "robo_collector_interfaces/msg/robot_move_type.hpp"
#include "robo_collector_interfaces/srv/get_current_coins.hpp"
#include "robo_collector_common/defines/RoboCollectorFunctionalDefines.h"
#include "game_engine/defines/ActionEventDefines.h"

//Own components headers

//Forward declarations

struct CollectorControllerExternalBridgeOutInterface {
  InvokeActionEventCb invokeActionEventCb;
  MoveButtonClickCb moveButtonClickCb;
};

class CollectorControllerExternalBridge: public rclcpp::Node {
public:
  CollectorControllerExternalBridge();

  int32_t init(const CollectorControllerExternalBridgeOutInterface &interface);

  void publishEnablePlayerInput();

private:
  typedef std_msgs::msg::Empty Empty;
  typedef robo_collector_interfaces::msg::RobotMoveType RobotMoveType;
  typedef robo_collector_interfaces::srv::GetCurrentCoins GetCurrentCoins;

  void onMoveMsg(const RobotMoveType::SharedPtr msg);

  //TODO remove after test
  void handleService(const std::shared_ptr<GetCurrentCoins::Request> request,
                     std::shared_ptr<GetCurrentCoins::Response> response);

  CollectorControllerExternalBridgeOutInterface _outInterface;

  rclcpp::Subscription<RobotMoveType>::SharedPtr _playerActSubscriber;
  rclcpp::Publisher<Empty>::SharedPtr _playerEnableInputPublisher;

  //TODO remove after test
  rclcpp::Service<GetCurrentCoins>::SharedPtr _getCoinsService;
  int64_t _coins = 0;
};

#endif /* ROBO_COLLECTOR_GUI_COLLECTORCONTROLLEREXTERNALBRIDGE_H_ */
