//Corresponding header
#include "ur_control_gui/config/UrControlGuiConfigGenerator.h"

//System headers

//Other libraries headers
#include <rclcpp/utilities.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "resource_utils/common/ResourceFileHeader.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "ur_control_gui/config/UrControlGuiConfig.h"
#include "ur_control_gui/external_api/UrControlGuiRos2ParamProvider.h"
#include "generated/UrControlGuiResources.h"

namespace {
constexpr auto PROJECT_FOLDER_NAME = "ur_control_gui";

enum TimerIds {

};

//UrControlGuiButtonHandlerConfig generateButtonHandlerConfig() {
//  UrControlGuiButtonHandlerConfig cfg;
//
//  cfg.moveButtonsRsrcIds = { UrControlGuiResources::UP_BUTTON,
//      UrControlGuiResources::LEFT_BUTTON, UrControlGuiResources::RIGHT_BUTTON };
//  cfg.moveButtonInfoTextFontId = UrControlGuiResources::VINQUE_RG_30;
//  cfg.helpButtonRsrcId = UrControlGuiResources::HELP_BUTTON;
//  cfg.settingsButtonRsrcId = UrControlGuiResources::SETTINGS_BUTTON;
//
//  return cfg;
//}

EngineConfig generateEngineConfig(const UrControlGuiRos2Params &rosParams) {
  const auto projectInstallPrefix =
      ament_index_cpp::get_package_share_directory(PROJECT_FOLDER_NAME);
  auto cfg = getDefaultEngineConfig(projectInstallPrefix);

  auto &windowCfg = cfg.managerHandlerCfg.drawMgrCfg.monitorWindowConfig;
  windowCfg.name = PROJECT_FOLDER_NAME;
  windowCfg.iconPath.append(projectInstallPrefix).append("/").append(
      ResourceFileHeader::getResourcesFolderName()).append(
      "/p/icons/UR_logo.png");
  windowCfg.pos = Point(rosParams.guiWindow.x, rosParams.guiWindow.y);
  windowCfg.width = rosParams.guiWindow.w;
  windowCfg.height = rosParams.guiWindow.h;
  windowCfg.displayMode = WindowDisplayMode::FULL_SCREEN;
  windowCfg.borderMode = WindowBorderMode::BORDERLESS;

  cfg.debugConsoleConfig.fontRsrcId = UrControlGuiResources::VINQUE_RG_30;

  return cfg;
}

UrControlGuiConfig generateGameConfig(const UrControlGuiRos2Params &rosParams) {
  UrControlGuiConfig cfg;
  cfg.robotIp = rosParams.robotIp;
  cfg.robotInterfacePort = rosParams.robotInterfacePort;

  auto &layoutCfg = cfg.layoutCfg;
  layoutCfg.mapRsrcId = UrControlGuiResources::BACKGROUND;

  return cfg;
}

} //end anonymous namespace

std::vector<DependencyDescription> UrControlGuiConfigGenerator::generateDependencies(
    int32_t argc, char **args) {
  std::vector<DependencyDescription> dependecies = getDefaultEngineDependencies(
      argc, args);

  const LoadDependencyCb ros2Loader = [argc, args]() {
    rclcpp::init(argc, args);
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

  auto paramProviderNode = std::make_shared<UrControlGuiRos2ParamProvider>();
  const auto rosParams = paramProviderNode->getParams();
  rosParams.print();

  cfg.engineCfg = generateEngineConfig(rosParams);
  cfg.gameCfg = generateGameConfig(rosParams);
  return cfg;
}

