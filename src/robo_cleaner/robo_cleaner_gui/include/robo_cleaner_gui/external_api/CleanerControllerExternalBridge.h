#ifndef ROBO_CLEANER_GUI_CLEANERCONTROLLEREXTERNALBRIDGE_H_
#define ROBO_CLEANER_GUI_CLEANERCONTROLLEREXTERNALBRIDGE_H_

//C system headers

//C++ system headers

//Other libraries headers
#include <rclcpp/node.hpp>
#include "game_engine/defines/ActionEventDefines.h"
#include "robo_common/defines/RoboCommonFunctionalDefines.h"
#include "robo_cleaner_interfaces/msg/robot_move_type.hpp"
#include "robo_cleaner_interfaces/srv/robot_move.hpp"

//Own components headers

//Forward declarations

struct CleanerControllerExternalBridgeOutInterface {
  InvokeActionEventCb invokeActionEventCb;
  RobotActCb robotActCb;
  SystemShutdownCb systemShutdownCb;
};

class CleanerControllerExternalBridge: public rclcpp::Node {
public:
  CleanerControllerExternalBridge();

  int32_t init(const CleanerControllerExternalBridgeOutInterface &interface);

  void publishShutdownController();

private:
  void onMoveMsg(
      const robo_cleaner_interfaces::msg::RobotMoveType::SharedPtr msg);

  //TODO remove after test
  void handleService(
      const std::shared_ptr<
      robo_cleaner_interfaces::srv::RobotMove::Request> request,
      std::shared_ptr<
      robo_cleaner_interfaces::srv::RobotMove::Response> response);

  CleanerControllerExternalBridgeOutInterface _outInterface;

  rclcpp::Subscription<robo_cleaner_interfaces::msg::RobotMoveType>::SharedPtr
    _playerDirSubscriber;

  rclcpp::Service<robo_cleaner_interfaces::srv::RobotMove>::SharedPtr
    _robotMoveService;
};

#endif /* ROBO_CLEANER_GUI_CLEANERCONTROLLEREXTERNALBRIDGE_H_ */
