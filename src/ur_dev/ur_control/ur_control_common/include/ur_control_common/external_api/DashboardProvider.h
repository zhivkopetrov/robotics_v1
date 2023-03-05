#ifndef UR_CONTROL_GUI_DASHBOARDPROVIDER_H_
#define UR_CONTROL_GUI_DASHBOARDPROVIDER_H_

//System headers
#include <thread>

//Other libraries headers
#include <rclcpp/node.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ur_dashboard_msgs/srv/get_robot_mode.hpp>
#include <ur_dashboard_msgs/srv/get_safety_mode.hpp>
#include "ur_control_common/defines/UrControlCommonFunctionalDefines.h"
#include "game_engine/defines/ActionEventDefines.h"
#include "utils/concurrency/ThreadSafeQueue.h"
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations

struct DashboardProviderOutInterface {
  InvokeActionEventCb invokeActionEventCb;
  RobotModeChangeCb robotModeChangeCb;
  SafetyModeChangeCb safetyModeChangeCb;
};

class DashboardProvider: public rclcpp::Node {
public:
  DashboardProvider();

  ErrorCode init(const DashboardProviderOutInterface& outInterface);
  void deinit();

  void invokeDashboard(DashboardCommand command);

private:
  ErrorCode initOutInterface(
      const DashboardProviderOutInterface &outInterface);
  ErrorCode initCommunication();

  using Trigger = std_srvs::srv::Trigger;
  using GetRobotMode = ur_dashboard_msgs::srv::GetRobotMode;
  using GetSafetyMode = ur_dashboard_msgs::srv::GetSafetyMode;

  void doRun();
  void invokeDashboardInternal(DashboardCommand command);

  void executeTriggerClient(const rclcpp::Client<Trigger>::SharedPtr& client);
  void getRobotMode();
  void getSafetyMode();

  DashboardProviderOutInterface _outInterface;

  std::thread _thread;
  ThreadSafeQueue<DashboardCommand> _commandQueue;

  rclcpp::Client<Trigger>::SharedPtr _powerOnService;
  rclcpp::Client<Trigger>::SharedPtr _powerOffService;
  rclcpp::Client<Trigger>::SharedPtr _brakeReleaseService;
  rclcpp::Client<GetRobotMode>::SharedPtr _getRobotModeService;
  rclcpp::Client<GetSafetyMode>::SharedPtr _getSafetyModeService;

  //Create reentrant callbacks groups so service calls
  //can be executed in parallel
  const rclcpp::CallbackGroup::SharedPtr _callbackGroup =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
};

#endif /* UR_CONTROL_GUI_DASHBOARDPROVIDER_H_ */
