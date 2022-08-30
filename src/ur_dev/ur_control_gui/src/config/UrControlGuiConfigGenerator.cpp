//Corresponding header
#include "ur_control_gui/config/UrControlGuiConfigGenerator.h"

//System headers

//Other libraries headers
#include <rclcpp/utilities.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "ros2_game_engine/communicator/config/Ros2CommunicatorConfig.h"
#include "resource_utils/common/ResourceFileHeader.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "ur_control_gui/config/UrControlGuiConfig.h"
#include "ur_control_gui/external_api/UrControlGuiRos2ParamProvider.h"
#include "generated/UrControlGuiResources.h"

namespace {
constexpr auto PROJECT_NAME = "ur_control_gui";
constexpr auto SCRIPTS_FOLDER_NAME = "scripts";

enum TimerIds {

};

ButtonHandlerConfig generateButtonHandlerConfig(
    const std::string &projectInstallPrefix) {
  ButtonHandlerConfig cfg;

  cfg.buttonRsrcId = UrControlGuiResources::UP_BUTTON;
  cfg.buttonFontRsrcId = UrControlGuiResources::VINQUE_RG_30;
  cfg.scriptFolderLocation = projectInstallPrefix;
  cfg.scriptFolderLocation.append("/").append(
      ResourceFileHeader::getResourcesFolderName().append("/").append(
          SCRIPTS_FOLDER_NAME));

  return cfg;
}

UrContolGuiExternalBridgeConfig generateUrContolGuiExternalBridgeConfig(
    const UrControlGuiRos2Params &rosParams) {
  UrContolGuiExternalBridgeConfig cfg;

  cfg.robotIp = rosParams.robotIp;
  cfg.robotInterfacePort = rosParams.robotInterfacePort;

  return cfg;
}

EngineConfig generateEngineConfig(const std::string &projectInstallPrefix,
                                  const UrControlGuiRos2Params &rosParams) {
  auto cfg = getDefaultEngineConfig(projectInstallPrefix);

  auto &windowCfg = cfg.managerHandlerCfg.drawMgrCfg.monitorWindowConfig;
  windowCfg.name = PROJECT_NAME;
  windowCfg.iconPath.append(projectInstallPrefix).append("/").append(
      ResourceFileHeader::getResourcesFolderName()).append(
      "/p/icons/UR_logo.png");
  windowCfg.pos = Point(rosParams.guiWindow.x, rosParams.guiWindow.y);
  windowCfg.width = rosParams.guiWindow.w;
  windowCfg.height = rosParams.guiWindow.h;
  windowCfg.displayMode = WindowDisplayMode::WINDOWED;
  windowCfg.borderMode = WindowBorderMode::BORDERLESS;

  cfg.debugConsoleConfig.fontRsrcId = UrControlGuiResources::VINQUE_RG_30;

  return cfg;
}

UrControlGuiConfig generateGameConfig(const std::string &projectInstallPrefix,
                                      const UrControlGuiRos2Params &rosParams) {
  UrControlGuiConfig cfg;
  cfg.urContolGuiExternalBridgeCfg = generateUrContolGuiExternalBridgeConfig(
      rosParams);

  auto &layoutCfg = cfg.layoutCfg;
  layoutCfg.buttonHandlerConfig = generateButtonHandlerConfig(
      projectInstallPrefix);

  layoutCfg.screenBoundary.w = rosParams.guiWindow.w;
  layoutCfg.screenBoundary.h = rosParams.guiWindow.h;
  layoutCfg.mapRsrcId = UrControlGuiResources::BACKGROUND;
  layoutCfg.robotImgRrscId = UrControlGuiResources::UR_ROBOT;
  layoutCfg.robotModeVisualsFontRsrcId = UrControlGuiResources::VINQUE_RG_45;

  return cfg;
}

Ros2CommunicatorConfig generateRos2CommunicatorConfig(
    const UrControlGuiRos2Params &rosParams) {
  const Ros2CommunicatorConfig cfg = rosParams.ros2CommunicatorConfig;
  return cfg;
}

} //end anonymous namespace

std::vector<DependencyDescription> UrControlGuiConfigGenerator::generateDependencies(
    int32_t argc, char **args) {
  std::vector<DependencyDescription> dependecies = getDefaultEngineDependencies(
      argc, args);

  const LoadDependencyCb ros2Loader = [argc, args]() {
    rclcpp::InitOptions initOptions;
    //leave the shutdown for user-side.
    //this will enable proper cleanup
    initOptions.shutdown_on_sigint = false;

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

  dependecies.push_back( { "ROS2", ros2Loader, ros2Unloader });

  return dependecies;
}

ApplicationConfig UrControlGuiConfigGenerator::generateConfig() {
  ApplicationConfig cfg;

  const auto projectInstallPrefix =
      ament_index_cpp::get_package_share_directory(PROJECT_NAME);
  auto paramProviderNode = std::make_shared<UrControlGuiRos2ParamProvider>();
  const auto rosParams = paramProviderNode->getParams();
  rosParams.print();

  cfg.engineCfg = generateEngineConfig(projectInstallPrefix, rosParams);
  cfg.gameCfg = generateGameConfig(projectInstallPrefix, rosParams);
  cfg.communicatorCfg = generateRos2CommunicatorConfig(rosParams);

  return cfg;
}

