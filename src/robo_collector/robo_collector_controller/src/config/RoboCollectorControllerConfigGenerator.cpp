//Corresponding header
#include "robo_collector_controller/config/RoboCollectorControllerConfigGenerator.h"

//C system headers

//C++ system headers

//Other libraries headers
#include <rclcpp/utilities.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "resource_utils/common/ResourceFileHeader.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_controller/config/RoboCollectorControllerConfig.h"
#include "generated/RoboCollectorControllerResources.h"

namespace {
//TODO parse the params from config
constexpr auto PROJECT_FOLDER_NAME = "robo_collector_controller";

//screen
constexpr auto WINDOW_X = 1272;
constexpr auto WINDOW_Y = 527;
constexpr auto WINDOW_WIDTH = 648;
constexpr auto WINDOW_HEIGHT = 553;

//misc
constexpr auto LOCAL_CONTROLLER_ENABLED = true;

enum TimerId {

};

RoboCollectorUiControllerBaseConfig generateRoboCollectorUiControllerConfig() {
  RoboCollectorUiControllerBaseConfig cfg;

  cfg.moveButtonsRsrcIds = { RoboCollectorControllerResources::UP_BUTTON,
      RoboCollectorControllerResources::LEFT_BUTTON,
      RoboCollectorControllerResources::RIGHT_BUTTON };
  cfg.moveButtonInfoTextFontId = RoboCollectorControllerResources::VINQUE_RG_30;
  cfg.horDelimiterRsrcId = RoboCollectorControllerResources::HOR_DELIMITER;
  cfg.vertDelimiterRsrcId = RoboCollectorControllerResources::VERT_DELIMITER;
  cfg.helpButtonRsrcId = RoboCollectorControllerResources::HELP_BUTTON;
  cfg.settingsButtonRsrcId = RoboCollectorControllerResources::SETTINGS_BUTTON;
  cfg.isEnabled = LOCAL_CONTROLLER_ENABLED;

  return cfg;
}

EngineConfig generateEngineConfig() {
  const auto projectInstallPrefix =
      ament_index_cpp::get_package_share_directory(PROJECT_FOLDER_NAME);
  auto cfg = getDefaultEngineConfig(projectInstallPrefix);

  auto &windowCfg = cfg.managerHandlerCfg.drawMgrCfg.monitorWindowConfig;
  windowCfg.name = PROJECT_FOLDER_NAME;
  windowCfg.iconPath.append(projectInstallPrefix).append("/").append(
      ResourceFileHeader::getResourcesFolderName()).append(
      "/p/icons/joystick.png");
  windowCfg.pos = Point(WINDOW_X, WINDOW_Y);
  windowCfg.width = WINDOW_WIDTH;
  windowCfg.height = WINDOW_HEIGHT;
  windowCfg.displayMode = WindowDisplayMode::WINDOWED;
  windowCfg.borderMode = WindowBorderMode::BORDERLESS;

  cfg.debugConsoleRsrcId = RoboCollectorControllerResources::VINQUE_RG_30;

  return cfg;
}

RoboCollectorControllerConfig generateGameConfig() {
  RoboCollectorControllerConfig cfg;
  auto& layoutCfg = cfg.layoutCfg;
  layoutCfg.uiControllerCfg = generateRoboCollectorUiControllerConfig();
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
    return SUCCESS;
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
  cfg.engineCfg = generateEngineConfig();
  cfg.gameCfg = generateGameConfig();
  return cfg;
}

