//Corresponding header
#include "urscript_bridge/config/UrBridgeConfigGenerator.h"

//System headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "urscript_bridge/external_api/UrBridgeRos2ParamProvider.h"

namespace {
UrBridgeExternalInterfaceConfig generetaUrBridgeExternalInterfaceConfig(
    const UrBridgeRos2Params& params) {
  UrBridgeExternalInterfaceConfig cfg;
  cfg.robotIp = params.robotIp;
  cfg.robotInterfacePort = params.robotInterfacePort;
  cfg.urScriptServiceReadyPin = params.urScriptServiceReadyPin;
  cfg.verboseLogging = params.verboseLogging;

  return cfg;
}

Ros2CommunicatorConfig generateRos2CommunicatorConfig(
    const UrBridgeRos2Params& params) {
  Ros2CommunicatorConfig cfg = params.ros2CommunicatorCfg;

  //block the main thread from the executor
  cfg.executionPolicy = ExecutionPolicy::BLOCKING;

  return cfg;
}

} //end anonymous namespace

std::vector<DependencyDescription> UrBridgeConfigGenerator::generateDependencies(
    int32_t argc, char **args) {
  const LoadDependencyCb ros2Loader = [argc, args]() {
    rclcpp::InitOptions initOptions;
    //SIGINT signal will cancel the executor and will unblock the main thread
    initOptions.shutdown_on_sigint = true;

    rclcpp::init(argc, args, initOptions);
    return ErrorCode::SUCCESS;
  };
  const UnloadDependencyCb ros2Unloader = []() {
    //shutdown the global context only if it hasn't
    //for example: ROS2 signal handlers do that automatically
    if (rclcpp::ok()) {
      const bool success = rclcpp::shutdown();
      if (!success) {
        LOGERR("Error, global context was already shutdowned");
      }
    }
  };

  const DependencyDescription dependency = { .name = "ROS2",
      .loadDependencyCb = ros2Loader, .unloadDependencyCb = ros2Unloader };

  return { dependency };
}

UrBridgeConfig UrBridgeConfigGenerator::generateConfig() {
  UrBridgeConfig cfg;

  auto paramProviderNode = std::make_shared<UrBridgeRos2ParamProvider>();
  const auto rosParams = paramProviderNode->getParams();
  rosParams.print();

  cfg.externalInterfaceConfig =
      generetaUrBridgeExternalInterfaceConfig(rosParams);
  cfg.ros2CommunicatorCfg = generateRos2CommunicatorConfig(rosParams);

  return cfg;
}

