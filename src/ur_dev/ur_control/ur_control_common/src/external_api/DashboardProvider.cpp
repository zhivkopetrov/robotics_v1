//Corresponding header
#include "ur_control_common/external_api/DashboardProvider.h"

//System headers

//Other libraries headers
#include "utils/data_type/EnumClassUtils.h"
#include "utils/log/Log.h"

//Own components headers
#include "ur_control_common/defines/UrControlCommonTopics.h"

namespace {
constexpr auto NODE_NAME = "UrDashboardProvider";

using namespace std::literals;

template <typename T>
void waitForService(T &client) {
  const char *serviceName = client->get_service_name();
  while (!client->wait_for_service(1s)) {
    LOG("Service [%s] not available. Waiting 1s ...", serviceName);
  }
}
} //end anonymous namespace

DashboardProvider::DashboardProvider()
    : Node(NODE_NAME) {

}

ErrorCode DashboardProvider::init(
    const DashboardProviderOutInterface &outInterface) {
  _thread = std::thread(&DashboardProvider::doRun, this);

  if (ErrorCode::SUCCESS != initOutInterface(outInterface)) {
    LOGERR("Error, initOutInterface() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initCommunication()) {
    LOGERR("Error,  initCommunication() failed");
    return ErrorCode::FAILURE;
  }

  invokeDashboard(DashboardCommand::GET_ROBOT_MODE);
  invokeDashboard(DashboardCommand::GET_SAFETY_MODE);

  return ErrorCode::SUCCESS;
}

void DashboardProvider::deinit() {
  _commandQueue.shutdown();
  _thread.join();
}

void DashboardProvider::invokeDashboard(DashboardCommand command) {
  _commandQueue.pushWithCopy(command);
}

ErrorCode DashboardProvider::initOutInterface(
    const DashboardProviderOutInterface &outInterface) {
  _outInterface = outInterface;
  if (nullptr == _outInterface.invokeActionEventCb) {
    LOGERR("Error, nullptr provided for InvokeActionEventCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.robotModeChangeCb) {
    LOGERR("Error, nullptr provided for RobotModeChangeCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.safetyModeChangeCb) {
    LOGERR("Error, nullptr provided for SafetyModeChangeCb");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode DashboardProvider::initCommunication() {
  _powerOnService = create_client<Trigger>(DASHBOARD_CLIENT_POWER_ON_SERVICE,
      rmw_qos_profile_services_default, _callbackGroup);

  _powerOffService = create_client<Trigger>(DASHBOARD_CLIENT_POWER_OFF_SERVICE,
      rmw_qos_profile_services_default, _callbackGroup);

  _brakeReleaseService = create_client<Trigger>(
      DASHBOARD_CLIENT_BRAKE_RELEASE_SERVICE, rmw_qos_profile_services_default,
      _callbackGroup);

  _getRobotModeService = create_client<GetRobotMode>(
      DASHBOARD_CLIENT_GET_ROBOT_MODE_SERVICE, rmw_qos_profile_services_default,
      _callbackGroup);

  _getSafetyModeService = create_client<GetSafetyMode>(
      DASHBOARD_CLIENT_GET_SAFETY_MODE_SERVICE,
      rmw_qos_profile_services_default, _callbackGroup);

  waitForService(_powerOnService);
  waitForService(_powerOffService);
  waitForService(_brakeReleaseService);
  waitForService(_getRobotModeService);
  waitForService(_getSafetyModeService);

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
    executeTriggerClient(_powerOnService);
    break;
  case DashboardCommand::POWER_OFF_ROBOT:
    executeTriggerClient(_powerOffService);
    break;
  case DashboardCommand::BRAKE_RELEASE:
    executeTriggerClient(_brakeReleaseService);
    break;
  case DashboardCommand::GET_ROBOT_MODE:
    getRobotMode();
    break;
  case DashboardCommand::GET_SAFETY_MODE:
    getSafetyMode();
    break;
  default:
    LOGERR("Error, received unsupported DashboardCommand: %d",
        getEnumValue(command));
    break;
  }
}

void DashboardProvider::executeTriggerClient(
    const rclcpp::Client<Trigger>::SharedPtr &client) {
  auto request = std::make_shared<Trigger::Request>();
  auto result = client->async_send_request(request);
  std::shared_ptr<Trigger::Response> response = result.get();

  if (!response->success) {
    LOGERR("Service call to [%s] failed. Reason: [%s]",
        client->get_service_name(), response->message.c_str());
    return;
  }
}

void DashboardProvider::getRobotMode() {
  auto request = std::make_shared<GetRobotMode::Request>();
  auto result = _getRobotModeService->async_send_request(request);
  std::shared_ptr<GetRobotMode::Response> response = result.get();

  if (!response->success) {
    LOGERR("Service call to [%s] failed",
        _getRobotModeService->get_service_name());
    return;
  }

  const RobotMode mode = toEnum<RobotMode>(response->robot_mode.mode);
  const auto f = [this, mode]() {
    _outInterface.robotModeChangeCb(mode);
  };

  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

void DashboardProvider::getSafetyMode() {
  auto request = std::make_shared<GetSafetyMode::Request>();
  auto result = _getSafetyModeService->async_send_request(request);
  std::shared_ptr<GetSafetyMode::Response> response = result.get();

  if (!response->success) {
    LOGERR("Service call to [%s] failed",
        _getSafetyModeService->get_service_name());
    return;
  }

  const SafetyMode mode = toEnum<SafetyMode>(response->safety_mode.mode);
  const auto f = [this, mode]() {
    _outInterface.safetyModeChangeCb(mode);
  };

  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}
