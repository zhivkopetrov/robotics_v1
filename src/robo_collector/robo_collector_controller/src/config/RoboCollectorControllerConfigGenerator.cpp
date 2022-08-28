//Corresponding header
#include "robo_collector_controller/config/RoboCollectorControllerConfigGenerator.h"

//System headers

//Other libraries headers
#include <rclcpp/utilities.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "ros2_game_engine/communicator/config/Ros2CommunicatorConfig.h"
#include "resource_utils/common/ResourceFileHeader.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_controller/config/RoboCollectorControllerConfig.h"
#include "robo_collector_controller/external_api/RoboCollectorControllerRos2ParamProvider.h"
#include "generated/RoboCollectorControllerResources.h"

namespace {
constexpr auto PROJECT_FOLDER_NAME = "robo_collector_controller";

enum TimerIds {
  INITIATE_USER_AUTHENTICATE_TIMER_ID
};

UserAuthenticateHelperConfig generateUserAuthenticateHelperConfig(
    const RoboCollectorControllerRos2Params& rosParams) {
  UserAuthenticateHelperConfig cfg;
  cfg.userData = rosParams.userData;
  cfg.timerId = INITIATE_USER_AUTHENTICATE_TIMER_ID;

  return cfg;
}

RoboCollectorUiControllerBaseConfig generateRoboCollectorUiControllerConfig(
    LocalControllerMode localControllerMode) {
  RoboCollectorUiControllerBaseConfig cfg;

  cfg.moveButtonsRsrcIds = { RoboCollectorControllerResources::UP_BUTTON,
      RoboCollectorControllerResources::LEFT_BUTTON,
      RoboCollectorControllerResources::RIGHT_BUTTON };
  cfg.moveButtonInfoTextFontId = RoboCollectorControllerResources::VINQUE_RG_30;
  cfg.horDelimiterRsrcId = RoboCollectorControllerResources::HOR_DELIMITER;
  cfg.vertDelimiterRsrcId = RoboCollectorControllerResources::VERT_DELIMITER;
  cfg.helpButtonRsrcId = RoboCollectorControllerResources::HELP_BUTTON;
  cfg.settingsButtonRsrcId = RoboCollectorControllerResources::SETTINGS_BUTTON;
  cfg.localControllerMode = localControllerMode;

  return cfg;
}

EngineConfig generateEngineConfig(
    const RoboCollectorControllerRos2Params& rosParams) {
  const auto projectInstallPrefix =
      ament_index_cpp::get_package_share_directory(PROJECT_FOLDER_NAME);
  auto cfg = getDefaultEngineConfig(projectInstallPrefix);

  auto &windowCfg = cfg.managerHandlerCfg.drawMgrCfg.monitorWindowConfig;
  windowCfg.name = PROJECT_FOLDER_NAME;
  windowCfg.iconPath.append(projectInstallPrefix).append("/").append(
      ResourceFileHeader::getResourcesFolderName()).append(
      "/p/icons/joystick.png");
  windowCfg.pos = Point(rosParams.guiWindow.x, rosParams.guiWindow.y);
  windowCfg.width = rosParams.guiWindow.w;
  windowCfg.height = rosParams.guiWindow.h;
  windowCfg.displayMode = WindowDisplayMode::WINDOWED;
  windowCfg.borderMode = WindowBorderMode::BORDERLESS;

  cfg.debugConsoleConfig.fontRsrcId =
      RoboCollectorControllerResources::VINQUE_RG_30;

  return cfg;
}

Ros2CommunicatorConfig generateRos2CommunicatorConfig(
    const RoboCollectorControllerRos2Params &rosParams) {
  const Ros2CommunicatorConfig cfg = rosParams.ros2CommunicatorConfig;
  return cfg;
}

RoboCollectorControllerConfig generateGameConfig(
    const RoboCollectorControllerRos2Params& rosParams) {
  RoboCollectorControllerConfig cfg;
  cfg.userAuthenticateHelperConfig =
      generateUserAuthenticateHelperConfig(rosParams);

  auto& layoutCfg = cfg.layoutCfg;
  layoutCfg.uiControllerCfg =
      generateRoboCollectorUiControllerConfig(rosParams.localControrllerMode);
  layoutCfg.mapRsrcId = RoboCollectorControllerResources::MAP;

  return cfg;
}

} //end anonymous namespace

std::vector<DependencyDescription>
RoboCollectorControllerConfigGenerator::generateDependencies(
    int32_t argc, char **args) {
  std::vector<DependencyDescription> dependecies =
      getDefaultEngineDependencies(argc, args);

  const LoadDependencyCb ros2Loader = [argc, args](){
    rclcpp::init(argc, args);
    return ErrorCode::SUCCESS;
  };
  const UnloadDependencyCb ros2Unloader = [](){
    //shutdown the global context only if it hasn't
    //for example: ROS2 signal handlers do that automatically
    if (rclcpp::ok()) {
      const bool success = rclcpp::shutdown();
      if (!success) {
        LOGERR("Error, global context was already shutdowned");
      }
    }
  };

  dependecies.push_back({"ROS2", ros2Loader, ros2Unloader});

  return dependecies;
}

ApplicationConfig RoboCollectorControllerConfigGenerator::generateConfig() {
  ApplicationConfig cfg;

  auto paramProviderNode =
      std::make_shared<RoboCollectorControllerRos2ParamProvider>();
  const auto rosParams = paramProviderNode->getParams();
  rosParams.print();

  cfg.engineCfg = generateEngineConfig(rosParams);
  cfg.gameCfg = generateGameConfig(rosParams);
  cfg.communicatorCfg = generateRos2CommunicatorConfig(rosParams);
  return cfg;
}

