//Corresponding header
#include "robo_miner_gui/config/RoboMinerGuiConfigGenerator.h"

//System headers
#include <numeric>

//Other libraries headers
#include <rclcpp/utilities.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "ros2_game_engine/communicator/config/Ros2CommunicatorConfig.h"
#include "robo_common/defines/RoboCommonDefines.h"
#include "robo_common/helpers/LevelFileLoader.h"
#include "resource_utils/common/ResourceFileHeader.h"
#include "utils/ErrorCode.h"
#include "utils/log/Log.h"

//Own components headers
#include "robo_miner_gui/config/RoboMinerGuiConfig.h"
#include "robo_miner_gui/defines/RoboMinerGuiDefines.h"
#include "robo_miner_gui/external_api/RoboMinerGuiRos2ParamProvider.h"
#include "generated/RoboMinerGuiResources.h"

namespace {
constexpr auto PROJECT_NAME = "robo_miner_gui";

enum TimerId {
  ROBOT_MOVE_ANIM_TIMER_ID,
  ROBOT_ROTATE_ANIM_TIMER_ID,
  ROBOT_COLLISION_ANIM_TIMER_ID,
  ROBOT_DAMAGE_ANIM_TIMER_ID,
  HEALTH_PANEL_MODIFY_INDICATOR_TIMER_ID,
  TILE_PANEL_INCR_TIMER_ID,
  TILE_PANEL_DECR_TIMER_ID,
  CRYSTAL_PANEL_INCR_TIMER_ID,
  CRYSTAL_PANEL_DECR_TIMER_ID,
  GAME_END_EXPAND_ANIM_TIMER_ID,
  GAME_END_FADE_ANIM_TIMER_ID,
  ACHIEVEMENT_FADE_AND_MODE_ANIM_TIMER_ID,
  COUNTDOWN_ANIM_TIMER_ID,
  HELP_PAGE_MOVE_AND_FADE_ANIM_TIMER_ID,

  FOG_OF_WAR_FADE_TIMER_IDS_START,

  //reserve 200 timers. Increase the number if needed

  FOG_OF_WAR_FADE_TIMER_IDS_END = FOG_OF_WAR_FADE_TIMER_IDS_START + 200
};

ObstacleHandlerConfig generateObstacleHandlerConfig() {
  ObstacleHandlerConfig cfg;

  cfg.obstacleRsrcId = RoboMinerGuiResources::MAP_OBSTACLE;
  cfg.status = ObstacleHandlerApproachOverlayStatus::DISABLED;

  return cfg;
}

RobotBaseConfig generateRobotBaseConfig() {
  RobotBaseConfig cfg;

  cfg.playerRsrcId = RoboMinerGuiResources::PLAYER_ROBOT;
  cfg.damageMarkerRsrcId = RoboMinerGuiResources::DAMAGE_MARKER;
  cfg.moveAnimStartTimerId = ROBOT_MOVE_ANIM_TIMER_ID;
  cfg.rotateAnimStartTimerId = ROBOT_ROTATE_ANIM_TIMER_ID;
  cfg.robotCollisionAnimStartTimerId = ROBOT_COLLISION_ANIM_TIMER_ID;
  cfg.robotDamageAnimStartTimerId = ROBOT_DAMAGE_ANIM_TIMER_ID;

  return cfg;
}

PanelHandlerConfig generatePanelHandlerConfig(int32_t emptyTilesCount,
                                              size_t longestCrystalSeqSize) {
  PanelHandlerConfig cfg;

  auto &healthPanelCfg = cfg.healthPanelCfg;
  healthPanelCfg.rsrcId = RoboMinerGuiResources::HEALTH_PANEL;
  healthPanelCfg.indicatorRsrcId = RoboMinerGuiResources::HEALTH_INDICATOR;
  healthPanelCfg.indicatorFontId = RoboMinerGuiResources::VINQUE_RG_30;
  healthPanelCfg.indicatorModifyTimerId =
      HEALTH_PANEL_MODIFY_INDICATOR_TIMER_ID;

  auto &tilePanelCfg = cfg.tilePanelCfg;
  tilePanelCfg.startValue = 1; //robot is starting from a valid tile
  tilePanelCfg.targetNumber = emptyTilesCount;
  tilePanelCfg.rsrcId = RoboMinerGuiResources::TILE_PANEL;
  tilePanelCfg.fontId = RoboMinerGuiResources::VINQUE_RG_75;
  tilePanelCfg.incrTimerId = TILE_PANEL_INCR_TIMER_ID;
  tilePanelCfg.decrTimerId = TILE_PANEL_DECR_TIMER_ID;

  auto &crystalPanelCfg = cfg.crystalPanelCfg;
  crystalPanelCfg.targetNumber = longestCrystalSeqSize;
  crystalPanelCfg.rsrcId = RoboMinerGuiResources::CRYSTAL_PANEL;
  crystalPanelCfg.fontId = RoboMinerGuiResources::VINQUE_RG_75;
  crystalPanelCfg.incrTimerId = CRYSTAL_PANEL_INCR_TIMER_ID;
  crystalPanelCfg.decrTimerId = CRYSTAL_PANEL_DECR_TIMER_ID;

  return cfg;
}

FieldConfig generateFieldConfig(const FieldDescription &fieldDescr,
                                FboOptimization fboOptimization) {
  FieldConfig cfg;

  cfg.description = fieldDescr;
  cfg.obstacleHandlerConfig = generateObstacleHandlerConfig();
  cfg.tileRsrcId = RoboMinerGuiResources::MAP_TILE;
  cfg.debugFontRsrcId = RoboMinerGuiResources::VINQUE_RG_30;
  cfg.fboOptimization = fboOptimization;

  return cfg;
}

FogOfWarConfig generateFogOfWarConfig(FogOfWarStatus status,
                                      const FieldPos &playerStartPos,
                                      const FieldDescription &fieldDescr) {
  FogOfWarConfig cfg;
  cfg.playerStartingPos = playerStartPos;
  cfg.status = status;
  cfg.cloudRsrcId = RoboMinerGuiResources::FOG_OF_WAR;

  const auto mapTilesCount = fieldDescr.rows * fieldDescr.cols;
  cfg.fogTilesFadeAnimTimerIds.resize(mapTilesCount);

  constexpr int startTimerId = FOG_OF_WAR_FADE_TIMER_IDS_START;
  std::iota(cfg.fogTilesFadeAnimTimerIds.begin(),
      cfg.fogTilesFadeAnimTimerIds.end(), startTimerId);

  return cfg;
}

GameEndAnimatorConfig generateGameEndAnimatorConfig(
    const RoboMinerGuiRos2Params &rosParams) {
  GameEndAnimatorConfig cfg;
  cfg.projectName = PROJECT_NAME;
  cfg.bgrRsrcId = RoboMinerGuiResources::MAP;
  cfg.winStatusFontId = RoboMinerGuiResources::VINQUE_RG_75;
  cfg.countdownFontId = RoboMinerGuiResources::VINQUE_RG_30;
  cfg.userDataFontId = RoboMinerGuiResources::VINQUE_RG_20;
  cfg.expandAnimTimerId = GAME_END_EXPAND_ANIM_TIMER_ID;
  cfg.fadeAnimTimerId = GAME_END_FADE_ANIM_TIMER_ID;
  cfg.countdownAnimTimerId = COUNTDOWN_ANIM_TIMER_ID;
  cfg.screenDimensions.w = rosParams.guiWindow.w;
  cfg.screenDimensions.h = rosParams.guiWindow.h;
  cfg.levelId = rosParams.levelId;

  return cfg;
}

AchievementAnimatorConfig generateAchievementAnimatorConfig(
    const RoboMinerGuiRos2Params &rosParams) {
  AchievementAnimatorConfig cfg;
  cfg.allStarsRsrcId = RoboMinerGuiResources::STARS;
  cfg.singleStarRsrcId = RoboMinerGuiResources::STAR_SINGLE;
  cfg.fadeAndMoveTimerId = ACHIEVEMENT_FADE_AND_MODE_ANIM_TIMER_ID;
  cfg.screenDimensions.w = rosParams.guiWindow.w;
  cfg.screenDimensions.h = rosParams.guiWindow.h;

  return cfg;
}

HelpPageAnimatorConfig generateHelpPageAnimatorConfig(
    const RoboMinerGuiRos2Params &rosParams) {
  HelpPageAnimatorConfig cfg;
  cfg.bgrToScreenRatio = 0.85;
  cfg.bgrRsrcId = RoboMinerGuiResources::MAP;
  cfg.moveAndFadeAnimTimerId = HELP_PAGE_MOVE_AND_FADE_ANIM_TIMER_ID;
  cfg.screenDimensions.w = rosParams.guiWindow.w;
  cfg.screenDimensions.h = rosParams.guiWindow.h;

  HelpPageEntry entry;
  entry.content = "Robo Miner Rules";
  entry.color = Colors::BLACK;
  entry.fontRsrcId = RoboMinerGuiResources::VINQUE_RG_75;
  cfg.titleEntry = entry;

  entry.fontRsrcId = RoboMinerGuiResources::VINQUE_RG_30;
  entry.content = "Objectives:";
  entry.color = Colors::BLUE;
  cfg.entries.push_back(entry);

  entry.color = Colors::BLACK;
  entry.content = "  - Stay in the map boundaries";
  cfg.entries.push_back(entry);

  entry.content = "  - Reveal the map and validate its content - yields 1 point";
  cfg.entries.push_back(entry);

  entry.content =
      "  - Validate the longest sequence of connected crystals - yields 1 point";
  cfg.entries.push_back(entry);

  entry.content =
      "  - Activate mining while being located on a tile from the longest crystal sequence";
  cfg.entries.push_back(entry);

  entry.content =
      "  - Mine all crystals from the longest crystal sequence - yields 1 point";
  cfg.entries.push_back(entry);

  entry.content =
      "      - Passing multiple times over the same tile while mining is active is allowed";
  cfg.entries.push_back(entry);

  entry.content = "  - Gathering all achievements on level ID 3 - yields 1 point";
  cfg.entries.push_back(entry);

  //create a boundary between objectives and lose conditions
  entry.prependedVerticalSpacing = 20;
  entry.color = Colors::RED;
  entry.content = "Lose conditions:";
  cfg.entries.push_back(entry);

  entry.prependedVerticalSpacing = 0; //reset the vertical spacing

  entry.color = Colors::BLACK;
  entry.content = "  - Query initial robot state more than once";
  cfg.entries.push_back(entry);

  entry.content = "  - Deplete Health indicator";
  cfg.entries.push_back(entry);

  entry.content = "  - 3 incorrect map validations";
  cfg.entries.push_back(entry);

  entry.content = "  - 3 incorrect longest crystal sequence validations";
  cfg.entries.push_back(entry);

  entry.content =
      "  - Activate mining while being outside of the longest crystal sequence tile";
  cfg.entries.push_back(entry);

  entry.content =
      "  - Step outside of longest crystal sequence tile while mining is active";
  cfg.entries.push_back(entry);

  return cfg;
}

SolutionValidatorConfig generateSolutionValidatorConfig(
    const std::string &projectInstallPrefix, int32_t emptyTilesCount,
    int32_t levelId, const FieldPos &playerStartPos) {
  SolutionValidatorConfig cfg;

  cfg.longestSequence = LevelFileLoader::readMinerLongestSolution(
      projectInstallPrefix, levelId);
  cfg.targetMapTilesCount = emptyTilesCount;
  cfg.playerStartLocation = playerStartPos;

  return cfg;
}

DebugFieldConfig generateDebugFieldConfig() {
  DebugFieldConfig cfg;
  cfg.panelRsrcId = RoboMinerGuiResources::MAP;
  cfg.texFotnRsrcId = RoboMinerGuiResources::VINQUE_RG_30;

  return cfg;
}

EngineConfig generateEngineConfig(const std::string &projectInstallPrefix,
                                  const RoboMinerGuiRos2Params &rosParams) {
  auto cfg = getDefaultEngineConfig(projectInstallPrefix);

  auto& drawMgrCfg = cfg.managerHandlerCfg.drawMgrCfg;
  auto& rendererCfg = drawMgrCfg.rendererConfig;
  rendererCfg.flagsMask = rosParams.rendererFlagsMask;
  rendererCfg.executionPolicy = rosParams.rendererExecutionPolicy;

  auto &sdlContainersCfg = cfg.managerHandlerCfg.sdlContainersCfg;
  sdlContainersCfg.maxResourceLoadingThreads = rosParams.resLoadingThreadsNum;

  auto &windowCfg = drawMgrCfg.monitorWindowConfig;
  windowCfg.name = PROJECT_NAME;
  windowCfg.iconPath.append(projectInstallPrefix).append("/").append(
      ResourceFileHeader::getResourcesFolderName()).append(
      "/p/entities/player_robot.png");
  windowCfg.pos = Point(rosParams.guiWindow.x, rosParams.guiWindow.y);
  windowCfg.width = rosParams.guiWindow.w;
  windowCfg.height = rosParams.guiWindow.h;
  windowCfg.displayMode = WindowDisplayMode::WINDOWED;
  windowCfg.borderMode = WindowBorderMode::BORDERLESS;

  cfg.debugConsoleConfig.maxFrameRate = rosParams.engineTargetFps;
  cfg.debugConsoleConfig.fontRsrcId = RoboMinerGuiResources::VINQUE_RG_30;

  return cfg;
}

RoboMinerGuiConfig generateGameConfig(const std::string &projectInstallPrefix,
                                      const RoboMinerGuiRos2Params &rosParams) {
  RoboMinerGuiConfig cfg;
  const auto [fieldDescr, initialRobotState] = LevelFileLoader::readLevelData(
      projectInstallPrefix, rosParams.levelId);

  cfg.solutionValidatorCfg = generateSolutionValidatorConfig(
      projectInstallPrefix, fieldDescr.emptyTilesCount, rosParams.levelId,
      initialRobotState.fieldPos);

  auto &layoutCfg = cfg.layoutCfg;
  layoutCfg.panelHandlerCfg = generatePanelHandlerConfig(
      fieldDescr.emptyTilesCount,
      cfg.solutionValidatorCfg.longestSequence.size());
  layoutCfg.crystalRsrcId = RoboMinerGuiResources::CRYSTALS;

  auto &commonLayoutCfg = layoutCfg.commonLayoutCfg;
  commonLayoutCfg.fieldCfg =
      generateFieldConfig(fieldDescr, rosParams.fboOptimization);
  commonLayoutCfg.robotInitialState = initialRobotState;
  commonLayoutCfg.robotBaseCfg = generateRobotBaseConfig();
  commonLayoutCfg.fogOfWarConfig = generateFogOfWarConfig(
      rosParams.fogOfWarStatus, initialRobotState.fieldPos, fieldDescr);
  commonLayoutCfg.gameEndAnimatorConfig = generateGameEndAnimatorConfig(
      rosParams);
  commonLayoutCfg.achievementAnimatorConfig = generateAchievementAnimatorConfig(
      rosParams);
  commonLayoutCfg.helpPageAnimatorConfig = generateHelpPageAnimatorConfig(
      rosParams);
  commonLayoutCfg.debugFieldConfig = generateDebugFieldConfig();
  commonLayoutCfg.mapRsrcId = RoboMinerGuiResources::MAP;
  commonLayoutCfg.playerFieldMarker = RoboCommonDefines::PLAYER_MARKER;

  return cfg;
}

Ros2CommunicatorConfig generateRos2CommunicatorConfig(
    const RoboMinerGuiRos2Params &rosParams) {
  const Ros2CommunicatorConfig cfg = rosParams.ros2CommunicatorConfig;
  return cfg;
}

} //end anonymous namespace

std::vector<DependencyDescription> RoboMinerGuiConfigGenerator::generateDependencies(
    int32_t argc, char **args) {
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

ApplicationConfig RoboMinerGuiConfigGenerator::generateConfig() {
  ApplicationConfig cfg;
  const auto projectInstallPrefix =
      ament_index_cpp::get_package_share_directory(PROJECT_NAME);

  auto paramProviderNode = std::make_shared<RoboMinerGuiRos2ParamProvider>();
  const auto rosParams = paramProviderNode->getParams();
  rosParams.print();

  cfg.engineCfg = generateEngineConfig(projectInstallPrefix, rosParams);
  cfg.gameCfg = generateGameConfig(projectInstallPrefix, rosParams);
  cfg.communicatorCfg = generateRos2CommunicatorConfig(rosParams);

  return cfg;
}

