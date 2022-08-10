//Corresponding header
#include "ur_control_gui/external_api/DashboardProvider.h"

//System headers

//Other libraries headers
#include "utils/data_type/EnumClassUtils.h"
#include "utils/Log.h"

//Own components headers
#include "ur_control_gui/defines/UrControlGuiTopics.h"

using namespace std::literals;

namespace {
constexpr auto NODE_NAME = "UrDashboardProvider";

template <typename T>
void waitForService(T &client) {
  const char *serviceName = client->get_service_name();
  while (!client->wait_for_service(1s)) {
    LOG("Service [%s] not available. Waiting 1s ...", serviceName);
  }
}
}

DashboardProvider::DashboardProvider()
    : Node(NODE_NAME) {

}

ErrorCode DashboardProvider::init() {
  _thread = std::thread(&DashboardProvider::doRun, this);

  if (ErrorCode::SUCCESS != initCommunication()) {
    LOGERR("Error,  initCommunication() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

void DashboardProvider::deinit() {
  _commandQueue.shutdown();
  _thread.join();
}

void DashboardProvider::invokeDashboard(DashboardCommand command) {
  _commandQueue.pushWithCopy(command);
}

ErrorCode DashboardProvider::initCommunication() {
  _powerOnService = create_client<Trigger>(DASHBOARD_CLIENT_POWER_ON_SERVICE);
  _powerOffService = create_client<Trigger>(DASHBOARD_CLIENT_POWER_OFF_SERVICE);
  _brakeReleaseService = create_client<Trigger>(
      DASHBOARD_CLIENT_BRAKE_RELEASE_SERVICE);

//  waitForService(_powerOnService);
//  waitForService(_powerOffService);
//  waitForService(_brakeReleaseService);

  return ErrorCode::SUCCESS;
}

void DashboardProvider::doRun() {
  DashboardCommand command;
  while (true) {
    const auto [isShutdowned, hasTimedOut] = _commandQueue.waitAndPop(command);
    if (isShutdowned) {
      return;
    }
    if (hasTimedOut) {
      continue;
    }

    invokeDashboardInternal(command);
  }
}

void DashboardProvider::invokeDashboardInternal(DashboardCommand command) {
  switch (command) {
  case DashboardCommand::POWER_ON_ROBOT:
    powerOn();
    break;
  case DashboardCommand::POWER_OFF_ROBOT:
    powerOff();
    break;
  case DashboardCommand::BRAKE_RELEASE:
    brakeRelease();
    break;
  case DashboardCommand::GET_ROBOT_MODE:
    getRobotMode();
    break;
  case DashboardCommand::GET_ROBOT_SAFETY_MODE:
    getSafetyMode();
    break;
  default:
    LOGERR("Error, received unsupported DashboardCommand: %d",
        getEnumValue(command));
    break;
  }
}

void DashboardProvider::powerOn() {
  auto request = std::make_shared<Trigger::Request>();
  auto result = _powerOnService->async_send_request(request);
  result.get();

//  if (rclcpp::spin_until_future_complete(shared_from_this(), result) !=
//      rclcpp::FutureReturnCode::SUCCESS) {
//    LOGERR("Failed to call service: [%s]", _powerOnService->get_service_name());
//    return;
//  }

  //handle success
}

void DashboardProvider::powerOff() {
  auto request = std::make_shared<Trigger::Request>();
  auto result = _powerOffService->async_send_request(request);
  result.get();

//  if (rclcpp::spin_until_future_complete(shared_from_this(), result) !=
//      rclcpp::FutureReturnCode::SUCCESS) {
//    LOGERR("Failed to call service: [%s]",
//        _powerOffService->get_service_name());
//    return;
//  }

  //handle success
}

void DashboardProvider::brakeRelease() {
  auto request = std::make_shared<Trigger::Request>();
  auto result = _brakeReleaseService->async_send_request(request);
  result.get();

//  if (rclcpp::spin_until_future_complete(shared_from_this(), result) !=
//      rclcpp::FutureReturnCode::SUCCESS) {
//    LOGERR("Failed to call service: [%s]",
//        _brakeReleaseService->get_service_name());
//    return;
//  }

  //handle success
}

void DashboardProvider::getRobotMode() {

}

void DashboardProvider::getSafetyMode() {

}
