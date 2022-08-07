#ifndef UR_CONTROL_GUI_DASHBOARDPROVIDER_H_
#define UR_CONTROL_GUI_DASHBOARDPROVIDER_H_

//System headers
#include <thread>

//Other libraries headers
#include <rclcpp/node.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "utils/concurrency/ThreadSafeQueue.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "ur_control_gui/defines/UrControlGuiDefines.h"

//Forward declarations

class DashboardProvider: public rclcpp::Node {
public:
  DashboardProvider();

  ErrorCode init();
  void deinit();

  void invokeDashboard(DashboardCommand command);

private:
  ErrorCode initCommunication();
  using Trigger = std_srvs::srv::Trigger;

  void doRun();
  void invokeDashboardInternal(DashboardCommand command);

  void powerOn();
  void powerOff();
  void brakeRelease();
  void getRobotMode();
  void getSafetyMode();

  std::thread _thread;
  ThreadSafeQueue<DashboardCommand> _commandQueue;

  rclcpp::Client<Trigger>::SharedPtr _powerOnService;
  rclcpp::Client<Trigger>::SharedPtr _powerOffService;
  rclcpp::Client<Trigger>::SharedPtr _brakeReleaseService;
};

#endif /* UR_CONTROL_GUI_DASHBOARDPROVIDER_H_ */
