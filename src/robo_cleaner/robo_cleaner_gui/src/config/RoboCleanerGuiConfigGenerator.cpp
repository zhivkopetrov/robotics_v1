#include "robo_cleaner_gui/config/RoboCleanerGuiConfigGenerator.h"

//System headers
#include <numeric>

//Other libraries headers
#include <rclcpp/utilities.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "robo_cleaner_common/defines/RoboCleanerDefines.h"
#include "robo_common/defines/RoboCommonDefines.h"
#include "robo_common/helpers/LevelFileLoader.h"
#include "resource_utils/common/ResourceFileHeader.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_cleaner_gui/config/RoboCleanerGuiConfig.h"
#include "robo_cleaner_gui/external_api/RoboCleanerGuiRos2ParamProvider.h"
#include "generated/RoboCleanerGuiResources.h"

namespace {
constexpr auto PROJECT_NAME = "robo_cleaner_gui";

enum TimerId {
  ROBOT_MOVE_ANIM_TIMER_ID,
  ROBOT_ROTATE_ANIM_TIMER_ID,
  ROBOT_COLLISION_ANIM_TIMER_ID,
  ROBOT_DAMAGE_ANIM_TIMER_ID,

  HEALTH_PANEL_MODIFY_INDICATOR_TIMER_ID,
  ENERGY_PANEL_MODIFY_INDICATOR_TIMER_ID,
  TILE_PANEL_INCR_TIMER_ID,
  TILE_PANEL_DECR_TIMER_ID,
  GAME_END_EXPAND_ANIM_TIMER_ID,
  GAME_END_FADE_ANIM_TIMER_ID,
  ACHIEVEMENT_FADE_AND_MODE_ANIM_TIMER_ID,
  COUNTDOWN_ANIM_TIMER_ID,
  HELP_PAGE_MOVE_AND_FADE_ANIM_TIMER_ID,
  RUBBISH_PANEL_INCR_TIMER_ID,
  RUBBISH_PANEL_DECR_TIMER_ID,

  FOG_OF_WAR_FADE_TIMER_IDS_START,

  //reserve 200 timers. Increase the number if needed

  FOG_OF_WAR_FADE_TIMER_IDS_END = FOG_OF_WAR_FADE_TIMER_IDS_START + 200
};

RobotBaseConfig generateRobotBaseConfig() {
  RobotBaseConfig cfg;

  cfg.playerRsrcId = RoboCleanerGuiResources::PLAYER_ROBOT;
  cfg.damageMarkerRsrcId = RoboCleanerGuiResources::DAMAGE_MARKER;
  cfg.moveAnimStartTimerId = ROBOT_MOVE_ANIM_TIMER_ID;
  cfg.rotateAnimStartTimerId = ROBOT_ROTATE_ANIM_TIMER_ID;
  cfg.robotCollisionAnimStartTimerId = ROBOT_COLLISION_ANIM_TIMER_ID;
  cfg.robotDamageAnimStartTimerId = ROBOT_DAMAGE_ANIM_TIMER_ID;

  return cfg;
}

PanelHandlerConfig generatePanelHandlerConfig(
    const FieldDescription &fieldDescr) {
  PanelHandlerConfig cfg;

  auto &tilePanelCfg = cfg.tilePanelCfg;
  tilePanelCfg.startValue = 1; //robot is starting from a valid tile
  tilePanelCfg.targetNumber = fieldDescr.emptyTilesCount;
  tilePanelCfg.rsrcId = RoboCleanerGuiResources::TILE_PANEL;
  tilePanelCfg.fontId = RoboCleanerGuiResources::VINQUE_RG_75;
  tilePanelCfg.incrTimerId = TILE_PANEL_INCR_TIMER_ID;
  tilePanelCfg.decrTimerId = TILE_PANEL_DECR_TIMER_ID;

  int32_t rubbishTilesCount { };
  for (const auto &row : fieldDescr.data) {
    for (const char marker : row) {
      if (isRubbishMarker(marker)) {
        rubbishTilesCount += getRubbishCounter(marker);
      }
    }
  }

  auto &rubbishPanelCfg = cfg.rubbishPanelCfg;
  rubbishPanelCfg.targetNumber = rubbishTilesCount;
  rubbishPanelCfg.rsrcId = RoboCleanerGuiResources::RUBBISH_PANEL;
  rubbishPanelCfg.fontId = RoboCleanerGuiResources::VINQUE_RG_75;
  rubbishPanelCfg.incrTimerId = RUBBISH_PANEL_INCR_TIMER_ID;
  rubbishPanelCfg.decrTimerId = RUBBISH_PANEL_DECR_TIMER_ID;

  auto &healthPanelCfg = cfg.healthPanelCfg;
  healthPanelCfg.rsrcId = RoboCleanerGuiResources::HEALTH_PANEL;
  healthPanelCfg.indicatorRsrcId = RoboCleanerGuiResources::HEALTH_INDICATOR;
  healthPanelCfg.indicatorFontId = RoboCleanerGuiResources::VINQUE_RG_30;
  healthPanelCfg.indicatorModifyTimerId =
      HEALTH_PANEL_MODIFY_INDICATOR_TIMER_ID;

  auto &energyPanelCfg = cfg.energyPanelCfg;
  energyPanelCfg.rsrcId = RoboCleanerGuiResources::ENERGY_PANEL;
  energyPanelCfg.indicatorRsrcId = RoboCleanerGuiResources::ENERGY_INDICATOR;
  energyPanelCfg.indicatorFontId = RoboCleanerGuiResources::VINQUE_RG_30;
  energyPanelCfg.indicatorModifyTimerId =
      ENERGY_PANEL_MODIFY_INDICATOR_TIMER_ID;

  return cfg;
}

ObstacleHandlerConfig generateObstacleHandlerConfig() {
  ObstacleHandlerConfig cfg;

  cfg.obstacleRsrcId = RoboCleanerGuiResources::MAP_OBSTACLE;
  cfg.status = ObstacleHandlerApproachOverlayStatus::ENABLED;

  return cfg;
}

RoboCleanerSolutionValidatorConfig generateSolutionValidatorConfig(
    const FieldPos &playerStartPos) {
  RoboCleanerSolutionValidatorConfig cfg;

  cfg.playerStartLocation = playerStartPos;

  return cfg;
}

EnergyHandlerConfig generateEnergyHandlerConfig(int32_t levelId) {
  EnergyHandlerConfig cfg;

  constexpr auto baseEnergyMoves = 30;
  constexpr auto additionalEnergyPerLevel = 30;
  cfg.maxMovesOnFullEnergy = baseEnergyMoves
      + (levelId * additionalEnergyPerLevel);

  return cfg;
}

FieldConfig generateFieldConfig(const FieldDescription &fieldDescr) {
  FieldConfig cfg;

  cfg.description = fieldDescr;
  cfg.obstacleHandlerConfig = generateObstacleHandlerConfig();
  cfg.tileRsrcId = RoboCleanerGuiResources::MAP_TILE;
  cfg.debugFontRsrcId = RoboCleanerGuiResources::VINQUE_RG_30;

  return cfg;
}

FogOfWarConfig generateFogOfWarConfig(FogOfWarStatus status,
                                      const FieldPos &playerStartPos,
                                      const FieldDescription &fieldDescr) {
  FogOfWarConfig cfg;
  cfg.playerStartingPos = playerStartPos;
  cfg.status = status;
  cfg.cloudRsrcId = RoboCleanerGuiResources::FOG_OF_WAR;

  const auto mapTilesCount = fieldDescr.rows * fieldDescr.cols;
  cfg.fogTilesFadeAnimTimerIds.resize(mapTilesCount);

  constexpr int32_t startTimerId = FOG_OF_WAR_FADE_TIMER_IDS_START;
  std::iota(cfg.fogTilesFadeAnimTimerIds.begin(),
      cfg.fogTilesFadeAnimTimerIds.end(), startTimerId);

  return cfg;
}

EntityHandlerConfig generateEntityHandlerConfig(
    const FieldPos &playerStartPos) {
  EntityHandlerConfig cfg;
  cfg.rubbishRsrcId = RoboCleanerGuiResources::RUBBISH;
  cfg.rubbishFontId = RoboCleanerGuiResources::VINQUE_RG_30;
  cfg.chargingStationRsrcId = RoboCleanerGuiResources::DAMAGE_MARKER;
  cfg.playerStartPosition = playerStartPos;

  return cfg;
}

GameEndAnimatorConfig generateGameEndAnimatorConfig(
    const RoboCleanerGuiRos2Params &rosParams) {
  GameEndAnimatorConfig cfg;
  cfg.projectName = PROJECT_NAME;
  cfg.bgrRsrcId = RoboCleanerGuiResources::MAP;
  cfg.winStatusFontId = RoboCleanerGuiResources::VINQUE_RG_75;
  cfg.countdownFontId = RoboCleanerGuiResources::VINQUE_RG_30;
  cfg.userDataFontId = RoboCleanerGuiResources::VINQUE_RG_20;
  cfg.expandAnimTimerId = GAME_END_EXPAND_ANIM_TIMER_ID;
  cfg.fadeAnimTimerId = GAME_END_FADE_ANIM_TIMER_ID;
  cfg.countdownAnimTimerId = COUNTDOWN_ANIM_TIMER_ID;
  cfg.screenDimensions.w = rosParams.guiWindow.w;
  cfg.screenDimensions.h = rosParams.guiWindow.h;
  cfg.levelId = rosParams.levelId;

  return cfg;
}

AchievementAnimatorConfig generateAchievementAnimatorConfig(
    const RoboCleanerGuiRos2Params &rosParams) {
  AchievementAnimatorConfig cfg;
  cfg.allStarsRsrcId = RoboCleanerGuiResources::STARS;
  cfg.singleStarRsrcId = RoboCleanerGuiResources::STAR_SINGLE;
  cfg.fadeAndMoveTimerId = ACHIEVEMENT_FADE_AND_MODE_ANIM_TIMER_ID;
  cfg.screenDimensions.w = rosParams.guiWindow.w;
  cfg.screenDimensions.h = rosParams.guiWindow.h;

  return cfg;
}

HelpPageAnimatorConfig generateHelpPageAnimatorConfig(
    const RoboCleanerGuiRos2Params &rosParams) {
  HelpPageAnimatorConfig cfg;
  cfg.bgrToScreenRatio = 0.95;
  cfg.bgrRsrcId = RoboCleanerGuiResources::MAP;
  cfg.moveAndFadeAnimTimerId = HELP_PAGE_MOVE_AND_FADE_ANIM_TIMER_ID;
  cfg.screenDimensions.w = rosParams.guiWindow.w;
  cfg.screenDimensions.h = rosParams.guiWindow.h;

  HelpPageEntry entry;
  entry.content = "Robo Cleaner Rules";
  entry.color = Colors::BLACK;
  entry.fontRsrcId = RoboCleanerGuiResources::VINQUE_RG_75;
  cfg.titleEntry = entry;

  entry.fontRsrcId = RoboCleanerGuiResources::VINQUE_RG_30;
  entry.content = "Objectives:";
  entry.color = Colors::BLUE;
  cfg.entries.push_back(entry);

  entry.color = Colors::BLACK;
  entry.content = "  - Stay in the map boundaries";
  cfg.entries.push_back(entry);

  entry.content = "  - Reveal the map - yields 1 point";
  cfg.entries.push_back(entry);

  entry.content = "  - Clean all rubbish from the map - yields 2 points";
  cfg.entries.push_back(entry);

  entry.content =
      "  - Finish the cleaning with a full health indicator - yields 1 point";
  cfg.entries.push_back(entry);

  entry.content =
      "  - Gathering all achievements on level ID 3 - yields 1 point";
  cfg.entries.push_back(entry);

  entry.content =
      "  - The top 3 players, who clean the map with the lowest total moves wins additional points:";
  cfg.entries.push_back(entry);

  entry.content =
      "    - 1st place (3 points), 2nd place (2 points), 3rd place (1 point)";
  cfg.entries.push_back(entry);

  entry.content = "  - Battery charging:";
  cfg.entries.push_back(entry);

  entry.content =
      "    - 1 turn spend on the charging station regenerates energy for 5 moves";
  cfg.entries.push_back(entry);

  entry.content =
      "    - you can specify how many turns to spend charging or 'charge_until_full'";
  cfg.entries.push_back(entry);

  entry.content =
      "    - if you run out of energy - an energy depleted penalty is applied";
  cfg.entries.push_back(entry);

  entry.content =
      "      - 50% of the total battery capacity is restored immediately and 'max_moves_on_full_energy' moves are wasted";
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

  entry.content = "  - 3 energy depleted penalties";
  cfg.entries.push_back(entry);

  return cfg;
}

DebugFieldConfig generateDebugFieldConfig() {
  DebugFieldConfig cfg;
  cfg.panelRsrcId = RoboCleanerGuiResources::MAP;
  cfg.texFotnRsrcId = RoboCleanerGuiResources::VINQUE_RG_30;

  return cfg;
}

EngineConfig generateEngineConfig(const std::string &projectInstallPrefix,
                                  const RoboCleanerGuiRos2Params &rosParams) {
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

  cfg.debugConsoleConfig.fontRsrcId = RoboCleanerGuiResources::VINQUE_RG_30;

  return cfg;
}

RoboCleanerGuiConfig generateGameConfig(
    const std::string &projectInstallPrefix,
    const RoboCleanerGuiRos2Params &rosParams) {
  RoboCleanerGuiConfig cfg;
  const auto [fieldDescr, initialRobotState] = LevelFileLoader::readLevelData(
      projectInstallPrefix, rosParams.levelId);

  cfg.solutionValidatorConfig = generateSolutionValidatorConfig(
      initialRobotState.fieldPos);
  cfg.energyHandlerConfig = generateEnergyHandlerConfig(rosParams.levelId);

  auto &layoutCfg = cfg.layoutCfg;
  layoutCfg.panelHandlerCfg = generatePanelHandlerConfig(fieldDescr);
  layoutCfg.entityHandlerCfg = generateEntityHandlerConfig(
      initialRobotState.fieldPos);

  auto &commonLayoutCfg = layoutCfg.commonLayoutCfg;
  commonLayoutCfg.fieldCfg = generateFieldConfig(fieldDescr);
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
  commonLayoutCfg.mapRsrcId = RoboCleanerGuiResources::MAP;
  commonLayoutCfg.playerFieldMarker = RoboCommonDefines::PLAYER_MARKER;

  return cfg;
}

Ros2CommunicatorConfig generateRos2CommunicatorConfig(
    const RoboCleanerGuiRos2Params &rosParams) {
  const Ros2CommunicatorConfig cfg = rosParams.ros2CommunicatorConfig;
  return cfg;
}

} //end anonymous namespace

std::vector<DependencyDescription> RoboCleanerGuiConfigGenerator::generateDependencies(
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

ApplicationConfig RoboCleanerGuiConfigGenerator::generateConfig() {
  ApplicationConfig cfg;
  const auto projectInstallPrefix =
      ament_index_cpp::get_package_share_directory(PROJECT_NAME);

  auto paramProviderNode = std::make_shared<RoboCleanerGuiRos2ParamProvider>();
  const auto rosParams = paramProviderNode->getParams();
  rosParams.print();

  cfg.engineCfg = generateEngineConfig(projectInstallPrefix, rosParams);
  cfg.gameCfg = generateGameConfig(projectInstallPrefix, rosParams);
  cfg.communicatorCfg = generateRos2CommunicatorConfig(rosParams);
  return cfg;
}

