//Corresponding headerur_control_bloomheaders
#include "ur_control_bloom/config/UrControlBloomConfigGenerator.h"

//System headers

//Other libraries headers
#include <rclcpp/utilities.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "ros2_game_engine/communicator/config/Ros2CommunicatorConfig.h"
#include "resource_utils/common/ResourceFileHeader.h"
#include "utils/ErrorCode.h"
#include "utils/log/Log.h"

//Own components headers
#include "ur_control_bloom/config/UrControlBloomConfig.h"
#include "ur_control_common/layout/entities/button_handler/config/CustomActionButtonHandlerConfig.h"
#include "ur_control_bloom/external_api/UrControlBloomRos2ParamProvider.h"
#include "generated/UrControlBloomResources.h"

namespace {
constexpr auto PROJECT_NAME = "ur_control_bloom";
constexpr auto SCRIPTS_FOLDER_NAME = "scripts";
constexpr auto GRIPPER_DEFINITIONS_FOLDER_NAME = "gripper_definitions";
constexpr auto CONFIG_FOLDER_NAME = "config";
constexpr auto STATE_FILE_NAME = "system_state.ini";
constexpr auto TOTAL_OBJECTS_PER_TOWER = 18;

//common waypoints
const AngleAxis HOME_ORIENTATION(0, 3.148, 0);
const WaypointJoint WAYPOINT_HOME_JOINT = 
  WaypointJoint({ -90, -90, -90, -90, 90, 0 });
const WaypointCartesian WAYPOINT_HOME_CARTESIAN = 
  WaypointCartesian(Point3d(-0.176, -0.691, 0.502), HOME_ORIENTATION);


ButtonHandlerHighLevelConfig generateButtonHandlerHighLevelConfig(
    const std::string &projectInstallPrefix) {
  CustomActionButtonHandlerConfig concreteCfg;

  concreteCfg.baseCfg.buttonRsrcId = UrControlBloomResources::UP_BUTTON;
  concreteCfg.baseCfg.buttonFontRsrcId = UrControlBloomResources::VINQUE_RG_30;
  
  std::string scriptsFolderLocation = projectInstallPrefix;
  scriptsFolderLocation.append("/").append(
      ResourceFileHeader::getResourcesFolderName().append("/").append(
          SCRIPTS_FOLDER_NAME)).append("/");

  auto& btnsDescr = concreteCfg.commandButtonsDescription;
  btnsDescr.resize(CUSTOM_ACTION_BUTTONS_COUNT);
  btnsDescr[PARK_IDX]             = { Point(100,  450), "Park" };
  btnsDescr[JENGA_IDX]            = { Point(100,  225), "Jenga" };
  btnsDescr[BLOOM_RANDOMIZED_IDX] = { Point(300,   25), "Bloom randomized" };
  btnsDescr[BLOOM_1ST_IDX]        = { Point(650,   25), "Bloom 1st" };
  btnsDescr[BLOOM_2ND_IDX]        = { Point(1000,  25), "Bloom 2nd" };
  btnsDescr[BLOOM_3RD_IDX]        = { Point(1350,  25), "Bloom 3rd" };
  btnsDescr[GRACEFUL_STOP_IDX]    = { Point(1630, 225), "Graceful stop" };
  btnsDescr[ABORT_MOTION_IDX]     = { Point(1630, 450), "Abort motion" };

  ButtonHandlerHighLevelConfig highLevelCfg;
  highLevelCfg.type = ButtonHandlerType::CUSTOM_ACTION;
  highLevelCfg.cfg = concreteCfg;
  return highLevelCfg;
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
    const UrControlBloomRos2Params &rosParams) {
  BloomMotionSequenceConfig cfg;
  cfg.endStrategy = rosParams.bloomEndStrategy;

  cfg.pickAndPlaceAcc = 1.0; // [m/s2]
  cfg.pickAndPlaceVel = 1.0; // [m/s]

  const AngleAxis ninetyHorizontalOrientation(2.224, -2.224, 0);
  const AngleAxis verticalOrientation(0, -2.224, 2.224);
  //TODO parse from files
  cfg.homeCartesian = WAYPOINT_HOME_CARTESIAN;
  cfg.graspApproachCartesian = 
    WaypointCartesian(Point3d(0.6, 0.145, 0.25), ninetyHorizontalOrientation);
  cfg.graspCartesian = 
    WaypointCartesian(Point3d(0.6, 0.145, -0.005), ninetyHorizontalOrientation);
  cfg.placeApproachCartesian = WaypointCartesian(
    Point3d(-0.196, -0.812, 0.6), verticalOrientation);
  cfg.placeCartesian = WaypointCartesian(
    Point3d(-0.196, -0.812, 0.236), verticalOrientation);

  cfg.homeJoint = WAYPOINT_HOME_JOINT;
  cfg.graspApproachJoint = 
    WaypointJoint({ 30.1, -86.3, -119.74, -64.23, 90.1, 30.12 });
  cfg.graspJoint = 
    WaypointJoint({ 30.05, -105.96, -131.78, -32.58, 89.97, 30.25 });
  cfg.placeApproachBasicStrategyJoint = 
    WaypointJoint({ -92.42, -94.62, -118.4, 33.02, 92.71, 0 });
  cfg.placeApproachFullRotationStrategyJoint = 
    WaypointJoint({ -92.42, -94.62, -118.4, 33.02, 92.71, 360 });

  cfg.twistStrategyWaypointOneJoint = 
    WaypointJoint({ 30.0, -86.33, -117.38, -66.6, 180.0, 30.22 });
  cfg.twistStrategyWaypointTwoJoint = 
    WaypointJoint({ -10.0, -82.22, -90.66, -97.43, 270.0, 30.22 });
  cfg.twistStrategyWaypointThreeJoint = 
    WaypointJoint({ -50.0, -82.34, -100.82, -102.60, 180.0, 14.62 });
  cfg.twistStrategyWaypointFourthJoint = cfg.placeApproachBasicStrategyJoint;

  return cfg;
}

JengaMotionSequenceConfig generateJengaMotionSequenceConfig(
    const UrControlBloomRos2Params &rosParams) {
  JengaMotionSequenceConfig cfg;
  cfg.endStrategy = rosParams.jengaEndStrategy;
  cfg.totalObjectsPerTower = TOTAL_OBJECTS_PER_TOWER;

  JengaBlockDimensions& blockDimensions = cfg.blockDimensions;
  blockDimensions.width = 0.120;  // [m]
  blockDimensions.depth = 0.040;  // [m]
  blockDimensions.height = 0.025; // [m]

  constexpr double gripperOpeningAddition = 0.015; // [m]

  //cummulative opening of 40 mm between gripper fingers
  constexpr int32_t m_to_mm_ratio = 1000;
  cfg.gripperOpening = static_cast<int32_t>(
    (blockDimensions.depth + gripperOpeningAddition) * m_to_mm_ratio);

  cfg.pickAndPlaceAcc = 1.0; // [m/s2]
  cfg.pickAndPlaceVel = 1.0; // [m/s]

  //TODO parse from files
  cfg.homeJoint = WAYPOINT_HOME_JOINT;
  cfg.graspApproachJoint = 
    WaypointJoint({ -34.79, -99.07, -82.88, -88.33, 90.22, 55.23 });

  cfg.zeroOrientation = HOME_ORIENTATION;
  cfg.ninetyOrientation = AngleAxis(2.221, 2.221, 0);
  cfg.homeCartesian = WAYPOINT_HOME_CARTESIAN;
  cfg.graspApproachCartesian = WaypointCartesian(
    Point3d(0.545, -0.592, 0.475), HOME_ORIENTATION);
  cfg.baseCenterACartesian = WaypointCartesian(
    Point3d(0.596, -0.426, -0.012), HOME_ORIENTATION);
  cfg.baseCenterBCartesian = WaypointCartesian(
    Point3d(0.495, -0.758, -0.012), HOME_ORIENTATION);

  return cfg;
}

ParkMotionSequenceConfig generateParkMotionSequenceConfig() {
  ParkMotionSequenceConfig cfg;

  cfg.motionAcc = 1.0; // [m/s2]
  cfg.motionVel = 1.0; // [m/s]

  //TODO parse from files
  cfg.homeCartesian = WAYPOINT_HOME_CARTESIAN;
  cfg.homeJoint = WAYPOINT_HOME_JOINT;

  return cfg;
}

UrScriptBuilderConfig generateUrScriptBuilderConfig(
  const std::string &projectInstallPrefix,
  const UrControlBloomRos2Params &rosParams) {
  UrScriptBuilderConfig cfg;

  std::string scriptsFolderLocation = projectInstallPrefix;
  scriptsFolderLocation.append("/").append(
      ResourceFileHeader::getResourcesFolderName().append("/").append(
          SCRIPTS_FOLDER_NAME)).append("/");

  cfg.gripperDefinitionFolder = 
    scriptsFolderLocation + GRIPPER_DEFINITIONS_FOLDER_NAME;

  cfg.gripperType = rosParams.gripperType;

  return cfg;
}

UrControlBloomMotionSequenceConfig generateUrControlBloomMotionSequenceConfig(
  const UrControlBloomRos2Params &rosParams) {
  UrControlBloomMotionSequenceConfig cfg;

  cfg.bloomMotionSequenceCfg = generateBloomMotionSequenceConfig(rosParams);
  cfg.jengaMotionSequenceCfg = generateJengaMotionSequenceConfig(rosParams);
  cfg.parkMotionSequenceCfg = generateParkMotionSequenceConfig();

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

std::string generateStateFilePath(const std::string &projectInstallPrefix) {
  std::string filePath = projectInstallPrefix;
  filePath.append("/").append(CONFIG_FOLDER_NAME).append("/")
          .append(STATE_FILE_NAME);

  return filePath;
}

UrControlBloomConfig generateGameConfig(
    const std::string &projectInstallPrefix,
    const UrControlBloomRos2Params &rosParams) {
  UrControlBloomConfig cfg;
  cfg.externalBridgeCfg = generateUrContolBloomExternalBridgeConfig(rosParams);
  cfg.motionSequenceCfg = generateUrControlBloomMotionSequenceConfig(rosParams);
  cfg.urScriptBuilderCfg = 
    generateUrScriptBuilderConfig(projectInstallPrefix, rosParams);
  cfg.stateFilePath = generateStateFilePath(projectInstallPrefix);

  auto &layoutCfg = cfg.layoutCfg;
  layoutCfg.roseRsrcId = UrControlBloomResources::ROSE;
  layoutCfg.jengaRsrcId = UrControlBloomResources::JENGA;
  layoutCfg.stateVisualsFontRsrcId = UrControlBloomResources::VINQUE_RG_45;

  auto &commonLayoutCfg = layoutCfg.commonLayoutCfg;
  commonLayoutCfg.buttonHandlerHighLevelCfg = 
    generateButtonHandlerHighLevelConfig(projectInstallPrefix);

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

