//Corresponding header
#include "robo_miner_gui/config/RoboMinerGuiConfigGenerator.h"

//System headers
#include <numeric>

//Other libraries headers
#include <rclcpp/utilities.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "robo_common/defines/RoboCommonDefines.h"
#include "robo_common/helpers/LevelFileLoader.h"
#include "resource_utils/common/ResourceFileHeader.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

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

FieldConfig generateFieldConfig(const FieldDescription& fieldDescr) {
  FieldConfig cfg;

  cfg.description = fieldDescr;
  cfg.obstacleHandlerConfig = generateObstacleHandlerConfig();
  cfg.tileRsrcId = RoboMinerGuiResources::MAP_TILE;
  cfg.debugFontRsrcId = RoboMinerGuiResources::VINQUE_RG_30;

  return cfg;
}

FogOfWarConfig generateFogOfWarConfig(FogOfWarStatus status,
                                      const FieldPos& playerStartPos,
                                      const FieldDescription& fieldDescr) {
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
    const RoboMinerGuiRos2Params& rosParams) {
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

  return cfg;
}

AchievementAnimatorConfig generateAchievementAnimatorConfig(
    const RoboMinerGuiRos2Params& rosParams) {
  AchievementAnimatorConfig cfg;
  cfg.allStarsRsrcId = RoboMinerGuiResources::STARS;
  cfg.singleStarRsrcId = RoboMinerGuiResources::STAR_SINGLE;
  cfg.fadeAndMoveTimerId = ACHIEVEMENT_FADE_AND_MODE_ANIM_TIMER_ID;
  cfg.screenDimensions.w = rosParams.guiWindow.w;
  cfg.screenDimensions.h = rosParams.guiWindow.h;

  return cfg;
}

SolutionValidatorConfig generateSolutionValidatorConfig(
    int32_t emptyTilesCount, int32_t levelId, const FieldPos& playerStartPos) {
  SolutionValidatorConfig cfg;

  const auto projectInstallPrefix =
      ament_index_cpp::get_package_share_directory(PROJECT_NAME);
  cfg.longestSequence = LevelFileLoader::readMinerLongestSolution(
      projectInstallPrefix, levelId);
  cfg.targetMapTilesCount = emptyTilesCount;
  cfg.playerStartLocation = playerStartPos;

  return cfg;
}

EngineConfig generateEngineConfig(const std::string& projectInstallPrefix,
                                  const RoboMinerGuiRos2Params& rosParams) {
  auto cfg = getDefaultEngineConfig(projectInstallPrefix);

  auto &windowCfg = cfg.managerHandlerCfg.drawMgrCfg.monitorWindowConfig;
  windowCfg.name = PROJECT_NAME;
  windowCfg.iconPath.append(projectInstallPrefix).append("/").append(
      ResourceFileHeader::getResourcesFolderName()).append(
      "/p/entities/player_robot.png");
  windowCfg.pos = Point(rosParams.guiWindow.x, rosParams.guiWindow.y);
  windowCfg.width = rosParams.guiWindow.w;
  windowCfg.height = rosParams.guiWindow.h;
  windowCfg.displayMode = WindowDisplayMode::WINDOWED;
  windowCfg.borderMode = WindowBorderMode::BORDERLESS;

  cfg.debugConsoleConfig.fontRsrcId = RoboMinerGuiResources::VINQUE_RG_30;

  return cfg;
}

RoboMinerGuiConfig generateGameConfig(const std::string& projectInstallPrefix,
                                      const RoboMinerGuiRos2Params& rosParams) {
  RoboMinerGuiConfig cfg;
  const auto [fieldDescr, initialRobotState] =
      LevelFileLoader::readLevelData(projectInstallPrefix, rosParams.levelId);

  cfg.solutionValidatorCfg = generateSolutionValidatorConfig(
      fieldDescr.emptyTilesCount, rosParams.levelId,
      initialRobotState.fieldPos);

  auto &layoutCfg = cfg.layoutCfg;
  layoutCfg.panelHandlerCfg = generatePanelHandlerConfig(
      fieldDescr.emptyTilesCount,
      cfg.solutionValidatorCfg.longestSequence.size());
  layoutCfg.crystalRsrcId = RoboMinerGuiResources::CRYSTALS;

  auto &commonLayoutCfg = layoutCfg.commonLayoutCfg;
  commonLayoutCfg.fieldCfg = generateFieldConfig(fieldDescr);
  commonLayoutCfg.robotInitialState = initialRobotState;
  commonLayoutCfg.robotBaseCfg = generateRobotBaseConfig();
  commonLayoutCfg.fogOfWarConfig = generateFogOfWarConfig(
      rosParams.fogOfWarStatus, initialRobotState.fieldPos, fieldDescr);
  commonLayoutCfg.gameEndAnimatorConfig =
      generateGameEndAnimatorConfig(rosParams);
  commonLayoutCfg.achievementAnimatorConfig =
      generateAchievementAnimatorConfig(rosParams);
  commonLayoutCfg.mapRsrcId = RoboMinerGuiResources::MAP;
  commonLayoutCfg.playerFieldMarker = RoboCommonDefines::PLAYER_MARKER;

  return cfg;
}

} //end anonymous namespace

std::vector<DependencyDescription> RoboMinerGuiConfigGenerator::generateDependencies(
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

ApplicationConfig RoboMinerGuiConfigGenerator::generateConfig() {
  ApplicationConfig cfg;
  const auto projectInstallPrefix =
      ament_index_cpp::get_package_share_directory(PROJECT_NAME);

  auto paramProviderNode = std::make_shared<RoboMinerGuiRos2ParamProvider>();
  const auto rosParams = paramProviderNode->getParams();
  rosParams.print();

  cfg.engineCfg = generateEngineConfig(projectInstallPrefix, rosParams);
  cfg.gameCfg = generateGameConfig(projectInstallPrefix, rosParams);
  return cfg;
}

