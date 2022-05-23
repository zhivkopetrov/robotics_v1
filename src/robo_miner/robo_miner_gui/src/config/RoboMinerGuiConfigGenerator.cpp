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
#include "generated/RoboMinerGuiResources.h"

namespace {
//TODO parse the params from config
constexpr auto PROJECT_FOLDER_NAME = "robo_miner_gui";

//screen
constexpr auto WINDOW_X = 72;
constexpr auto WINDOW_Y = 27;
constexpr auto WINDOW_WIDTH = 1848;
constexpr auto WINDOW_HEIGHT = 1053;

//misc
constexpr auto ROBOT_FIELD_MARKERS = RobotFieldMarkers::DISABLED;
constexpr auto LEVEL_ID = 1;

enum TimerId {
  ROBOT_MOVE_ANIM_TIMER_ID,
  ROBOT_WALL_COLLISION_ANIM_TIMER_ID,
  ROBOT_COLLISION_ANIM_TIMER_ID,
  ROBOT_DAMAGE_ANIM_TIMER_ID,
  HEALTH_PANEL_REDUCE_INDICATOR_TIMER_ID,
  TILE_PANEL_INCR_TIMER_ID,
  TILE_PANEL_DECR_TIMER_ID,
  CRYSTAL_PANEL_INCR_TIMER_ID,
  CRYSTAL_PANEL_DECR_TIMER_ID,

  FOG_OF_WAR_FADE_TIMER_IDS_START,

  //reserve 200 timers. Increase the number if needed

  FOG_OF_WAR_FADE_TIMER_IDS_END = FOG_OF_WAR_FADE_TIMER_IDS_START + 200
};

ObstacleHandlerConfig generateObstacleHandlerConfig() {
  ObstacleHandlerConfig cfg;

  cfg.obstacleRsrcId = RoboMinerGuiResources::MAP_OBSTACLE;

  return cfg;
}

RobotBaseConfig generateRobotBaseConfig() {
  RobotBaseConfig cfg;

  cfg.playerRsrcId = RoboMinerGuiResources::PLAYER_ROBOT;
  cfg.damageMarkerRsrcId = RoboMinerGuiResources::DAMAGE_MARKER;
  cfg.moveAnimStartTimerId = ROBOT_MOVE_ANIM_TIMER_ID;
  cfg.wallCollisionAnimStartTimerId = ROBOT_WALL_COLLISION_ANIM_TIMER_ID;
  cfg.robotCollisionAnimStartTimerId = ROBOT_COLLISION_ANIM_TIMER_ID;
  cfg.robotDamageAnimStartTimerId = ROBOT_DAMAGE_ANIM_TIMER_ID;
  cfg.robotFieldMarkers = ROBOT_FIELD_MARKERS;

  return cfg;
}

PanelHandlerConfig generatePanelHandlerConfig(int32_t emptyTilesCount,
                                              size_t longestCrystalSeqSize) {
  PanelHandlerConfig cfg;

  auto &healthPanelCfg = cfg.healthPanelCfg;
  healthPanelCfg.rsrcId = RoboMinerGuiResources::HEALTH_PANEL;
  healthPanelCfg.indicatorRsrcId = RoboMinerGuiResources::HEALTH_INDICATOR;
  healthPanelCfg.indicatorFontId = RoboMinerGuiResources::VINQUE_RG_30;
  healthPanelCfg.indicatorReduceTimerId =
      HEALTH_PANEL_REDUCE_INDICATOR_TIMER_ID;

  auto &tilePanelCfg = cfg.tilePanelCfg;
  tilePanelCfg.startValue = 1; //robot is starting from a valid tile
  tilePanelCfg.targetNumber = emptyTilesCount - 30;
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

FieldConfig generateFieldConfig() {
  FieldConfig cfg;

  const auto projectInstallPrefix =
      ament_index_cpp::get_package_share_directory(PROJECT_FOLDER_NAME);
  cfg.description = LevelFileLoader::readFieldDescription(projectInstallPrefix,
      LEVEL_ID);

  cfg.obstacleHandlerConfig = generateObstacleHandlerConfig();

  cfg.tileRsrcId = RoboMinerGuiResources::MAP_TILE;
  cfg.debugFontRsrcId = RoboMinerGuiResources::VINQUE_RG_30;

  return cfg;
}

FogOfWarConfig generateFogOfWarConfig(int32_t mapTilesCount) {
  FogOfWarConfig cfg;
  cfg.status = FogOfWarStatus::ENABLED;
  cfg.cloudRsrcId = RoboMinerGuiResources::FOG_OF_WAR;

  constexpr int startTimerId = FOG_OF_WAR_FADE_TIMER_IDS_START;
  cfg.fogTilesFadeAnimTimerIds.resize(mapTilesCount);
  std::iota(cfg.fogTilesFadeAnimTimerIds.begin(),
      cfg.fogTilesFadeAnimTimerIds.end(), startTimerId);

  return cfg;
}

SolutionValidatorConfig generateSolutionValidatorConfig(
    const FieldDescription& fieldDescr) {
  SolutionValidatorConfig cfg;

  const auto projectInstallPrefix =
      ament_index_cpp::get_package_share_directory(PROJECT_FOLDER_NAME);
  cfg.longestSequence = LevelFileLoader::readMinerLongestSolution(
      projectInstallPrefix, LEVEL_ID);
  cfg.targetMapTilesCount = fieldDescr.emptyTilesCount;
  cfg.playerStartLocation.row = fieldDescr.rows - 1;
  cfg.playerStartLocation.col = fieldDescr.cols - 1;

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
      "/p/entities/player_robot.png");
  windowCfg.pos = Point(WINDOW_X, WINDOW_Y);
  windowCfg.width = WINDOW_WIDTH;
  windowCfg.height = WINDOW_HEIGHT;
  windowCfg.displayMode = WindowDisplayMode::WINDOWED;
  windowCfg.borderMode = WindowBorderMode::BORDERLESS;

  cfg.debugConsoleRsrcId = RoboMinerGuiResources::VINQUE_RG_30;

  return cfg;
}

RoboMinerGuiConfig generateGameConfig() {
  RoboMinerGuiConfig cfg;

  auto &layoutCfg = cfg.layoutCfg;

  auto &commonLayoutCfg = layoutCfg.commonLayoutCfg;
  commonLayoutCfg.fieldCfg = generateFieldConfig();
  const auto mapTilesCount = commonLayoutCfg.fieldCfg.description.rows
      * commonLayoutCfg.fieldCfg.description.cols;
  const auto emptyTilesCount =
      commonLayoutCfg.fieldCfg.description.emptyTilesCount;

  commonLayoutCfg.fogOfWarConfig = generateFogOfWarConfig(mapTilesCount);
  commonLayoutCfg.robotBaseCfg = generateRobotBaseConfig();
  commonLayoutCfg.mapRsrcId = RoboMinerGuiResources::MAP;
  commonLayoutCfg.playerFieldMarker = RoboCommonDefines::PLAYER_MARKER;

  cfg.solutionValidatorCfg =
      generateSolutionValidatorConfig(commonLayoutCfg.fieldCfg.description);

  layoutCfg.panelHandlerCfg = generatePanelHandlerConfig(emptyTilesCount,
      cfg.solutionValidatorCfg.longestSequence.size());
  layoutCfg.crystalRsrcId = RoboMinerGuiResources::CRYSTALS;

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
  cfg.engineCfg = generateEngineConfig();
  cfg.gameCfg = generateGameConfig();
  return cfg;
}

