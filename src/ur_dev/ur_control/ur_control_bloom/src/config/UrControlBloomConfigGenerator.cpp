//Corresponding headerur_control_bloomheaders
#include "ur_control_bloom/config/UrControlBloomConfigGenerator.h"

//System headers

//Other libraries headers
#include <rclcpp/utilities.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "ros2_game_engine/communicator/config/Ros2CommunicatorConfig.h"
#include "resource_utils/common/ResourceFileHeader.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "ur_control_bloom/config/UrControlBloomConfig.h"
#include "ur_control_bloom/external_api/UrControlBloomRos2ParamProvider.h"
#include "generated/UrControlBloomResources.h"

namespace {
constexpr auto PROJECT_NAME = "ur_control_bloom";
constexpr auto SCRIPTS_FOLDER_NAME = "scripts";
constexpr auto GRIPPER_SCRIPTS_FOLDER_NAME = "gripper";
constexpr auto COMMAND_SCRIPTS_FOLDER_NAME = "command";

enum TimerIds {

};

ButtonHandlerConfig generateButtonHandlerConfig(
    const std::string &projectInstallPrefix) {
  ButtonHandlerConfig cfg;

  cfg.buttonRsrcId = UrControlBloomResources::UP_BUTTON;
  cfg.buttonFontRsrcId = UrControlBloomResources::VINQUE_RG_30;
  
  std::string scriptsFolderLocation = projectInstallPrefix;
  scriptsFolderLocation.append("/").append(
      ResourceFileHeader::getResourcesFolderName().append("/").append(
          SCRIPTS_FOLDER_NAME)).append("/");

  cfg.gripperScriptFolderLocation = 
    scriptsFolderLocation + GRIPPER_SCRIPTS_FOLDER_NAME;
  cfg.commandScriptsFolderLocation = 
    scriptsFolderLocation + COMMAND_SCRIPTS_FOLDER_NAME;

  cfg.commandButtonsDescription = {
    { Point(100,  450), "Jenga" },
    { Point(100,  225), "Bloom randomized" },
    { Point(300,   25), "Bloom 1st" },
    { Point(650,   25), "Bloom 2nd" },
    { Point(1000,  25), "Bloom 3rd" },
    { Point(1370,  25), "Bloom 4th" },
    { Point(1545, 225), "Preempt motion" },
    { Point(1545, 450), "Return home" }
  };

  return cfg;
}

UrContolBloomExternalBridgeConfig generateUrContolBloomExternalBridgeConfig(
    const UrControlBloomRos2Params &rosParams) {
  UrContolBloomExternalBridgeConfig cfg;
  auto& commonCfg = cfg.commonConfig;
  commonCfg.robotIp = rosParams.robotIp;
  commonCfg.robotInterfacePort = rosParams.robotInterfacePort;

  return cfg;
}

BloomMotionSequenceConfig generateBloomMotionSequenceConfig(
    [[maybe_unused]]const UrControlBloomRos2Params &rosParams) {
  BloomMotionSequenceConfig cfg;
  //TODO parse from files
  cfg.homeJoint = WaypointJoint({ -90, -90, -90, -90, 90, 0 });
  cfg.homeCartesian = 
    WaypointCartesian(Point3d(-0.176, -0.691, 0.502), AngleAxis(0, 3.148, 0));

  cfg.graspJoint = 
    WaypointJoint({ 30.05, -105.96, -131.78, -32.58, 89.97, 30.25 });
  cfg.graspApproachJoint = 
    WaypointJoint({ 30.1, -86.3, -119.74, -64.23, 90.1, 30.12 });
  cfg.placeJoint = 
    WaypointJoint({ -92.42, -127.52, -126.75, 74.27, 91.99, 0 });
  cfg.placeApproachJoint = 
    WaypointJoint({ -92.42, -94.62, -118.4, 33.02, 92.21, 0 });

  return cfg;
}

JengaMotionSequenceConfig generateJengaMotionSequenceConfig(
    [[maybe_unused]]const UrControlBloomRos2Params &rosParams) {
  JengaMotionSequenceConfig cfg;
  //TODO parse from files
  const AngleAxis orientation(2.224, -2.224, 0.0);
  cfg.homeCartesian = 
    WaypointCartesian(Point3d(-0.176, -0.691, 0.502), AngleAxis(0, 3.148, 0));

  cfg.graspApproachCartesian = WaypointCartesian(
    Point3d(0.596, -0.426, -0.04), AngleAxis(0.994, 2.991, 0));
  cfg.baseCenterACartesian = WaypointCartesian(
    Point3d(0.495, -0.758, -0.04), AngleAxis(1.403, -2.799, -0.009));
  cfg.baseCenterBCartesian = WaypointCartesian(
    Point3d(0.55, -0.055, 0.25), AngleAxis(0.994, 2.991, 0));

  return cfg;
}

EngineConfig generateEngineConfig(const std::string &projectInstallPrefix,
                                  const UrControlBloomRos2Params &rosParams) {
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

  cfg.debugConsoleConfig.fontRsrcId = UrControlBloomResources::VINQUE_RG_30;

  return cfg;
}

UrControlBloomConfig generateGameConfig(
    const std::string &projectInstallPrefix,
    const UrControlBloomRos2Params &rosParams) {
  UrControlBloomConfig cfg;
  cfg.externalBridgeCfg = generateUrContolBloomExternalBridgeConfig(rosParams);
  cfg.bloomMotionSequenceCfg = generateBloomMotionSequenceConfig(rosParams);
  cfg.jengaMotionSequenceCfg = generateJengaMotionSequenceConfig(rosParams);

  auto &layoutCfg = cfg.layoutCfg;
  layoutCfg.roseRsrcId = UrControlBloomResources::ROSE;
  layoutCfg.jengaRsrcId = UrControlBloomResources::JENGA;
  layoutCfg.stateVisualsFontRsrcId = UrControlBloomResources::VINQUE_RG_45;

  auto &commonLayoutCfg = layoutCfg.commonLayoutCfg;
  commonLayoutCfg.buttonHandlerConfig = generateButtonHandlerConfig(
      projectInstallPrefix);

  commonLayoutCfg.screenBoundary.w = rosParams.guiWindow.w;
  commonLayoutCfg.screenBoundary.h = rosParams.guiWindow.h;
  commonLayoutCfg.mapRsrcId = UrControlBloomResources::BACKGROUND;
  commonLayoutCfg.robotImgRrscId = UrControlBloomResources::UR_ROBOT;
  commonLayoutCfg.robotModeVisualsFontRsrcId = 
    UrControlBloomResources::VINQUE_RG_45;

  return cfg;
}

Ros2CommunicatorConfig generateRos2CommunicatorConfig(
    const UrControlBloomRos2Params &rosParams) {
  const Ros2CommunicatorConfig cfg = rosParams.ros2CommunicatorConfig;
  return cfg;
}

} //end anonymous namespace

std::vector<DependencyDescription> 
UrControlBloomConfigGenerator::generateDependencies(int32_t argc, char **args) {
  std::vector<DependencyDescription> dependecies = getDefaultEngineDependencies(
      argc, args);

  const LoadDependencyCb ros2Loader = [argc, args]() {
    rclcpp::InitOptions initOptions;
    //leave the shutdown for user-side.
    //this will enable proper cleanup
    initOptions.shutdown_on_signal = false;

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

ApplicationConfig UrControlBloomConfigGenerator::generateConfig() {
  ApplicationConfig cfg;

  const auto projectInstallPrefix =
      ament_index_cpp::get_package_share_directory(PROJECT_NAME);
  auto paramProviderNode = std::make_shared<UrControlBloomRos2ParamProvider>();
  const auto rosParams = paramProviderNode->getParams();
  rosParams.print();

  cfg.engineCfg = generateEngineConfig(projectInstallPrefix, rosParams);
  cfg.gameCfg = generateGameConfig(projectInstallPrefix, rosParams);
  cfg.communicatorCfg = generateRos2CommunicatorConfig(rosParams);

  return cfg;
}

