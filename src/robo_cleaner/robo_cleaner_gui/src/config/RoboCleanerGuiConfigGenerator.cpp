//Corresponding header
#include "robo_cleaner_gui/config/RoboCleanerGuiConfigGenerator.h"

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
#include "robo_cleaner_gui/config/RoboCleanerGuiConfig.h"
#include "robo_cleaner_gui/defines/RoboCleanerGuiDefines.h"
#include "generated/RoboCleanerGuiResources.h"

namespace {
//TODO parse the params from config
constexpr auto PROJECT_FOLDER_NAME = "robo_cleaner_gui";

//screen
constexpr auto WINDOW_X = 72;
constexpr auto WINDOW_Y = 27;
constexpr auto WINDOW_WIDTH = 1848;
constexpr auto WINDOW_HEIGHT = 1053;

//misc
constexpr auto ROBOT_FIELD_MARKERS = RobotFieldMarkers::DISABLED;

//TODO compute from the field config
constexpr auto RUBBISH_CLEANS_CTN = 63;

enum TimerId {
  ROBOT_MOVE_ANIM_TIMER_ID,
  ROBOT_WALL_COLLISION_ANIM_TIMER_ID,
  ROBOT_COLLISION_ANIM_TIMER_ID,
  ROBOT_DAMAGE_ANIM_TIMER_ID,

  HEALTH_PANEL_REDUCE_INDICATOR_TIMER_ID,
  ENERGY_PANEL_REDUCE_INDICATOR_TIMER_ID,
  TILE_PANEL_INCR_TIMER_ID,
  TILE_PANEL_DECR_TIMER_ID,
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
  cfg.wallCollisionAnimStartTimerId = ROBOT_WALL_COLLISION_ANIM_TIMER_ID;
  cfg.robotCollisionAnimStartTimerId = ROBOT_COLLISION_ANIM_TIMER_ID;
  cfg.robotDamageAnimStartTimerId = ROBOT_DAMAGE_ANIM_TIMER_ID;
  cfg.robotFieldMarkers = ROBOT_FIELD_MARKERS;

  return cfg;
}

PanelHandlerConfig generatePanelHandlerConfig(int32_t mapTilesCount) {
  PanelHandlerConfig cfg;

  auto &tilePanelCfg = cfg.tilePanelCfg;
  tilePanelCfg.startValue = 1; //robot is starting from a valid tile
  tilePanelCfg.targetNumber = mapTilesCount;
  tilePanelCfg.rsrcId = RoboCleanerGuiResources::TILE_PANEL;
  tilePanelCfg.fontId = RoboCleanerGuiResources::VINQUE_RG_75;
  tilePanelCfg.incrTimerId = TILE_PANEL_INCR_TIMER_ID;
  tilePanelCfg.decrTimerId = TILE_PANEL_DECR_TIMER_ID;

  auto &rubbishPanelCfg = cfg.rubbishPanelCfg;
  rubbishPanelCfg.targetNumber = RUBBISH_CLEANS_CTN;
  rubbishPanelCfg.rsrcId = RoboCleanerGuiResources::RUBBISH_PANEL;
  rubbishPanelCfg.fontId = RoboCleanerGuiResources::VINQUE_RG_75;
  rubbishPanelCfg.incrTimerId = RUBBISH_PANEL_INCR_TIMER_ID;
  rubbishPanelCfg.decrTimerId = RUBBISH_PANEL_DECR_TIMER_ID;

  auto &healthPanelCfg = cfg.healthPanelCfg;
  healthPanelCfg.rsrcId = RoboCleanerGuiResources::HEALTH_PANEL;
  healthPanelCfg.indicatorRsrcId = RoboCleanerGuiResources::HEALTH_INDICATOR;
  healthPanelCfg.indicatorFontId = RoboCleanerGuiResources::VINQUE_RG_30;
  healthPanelCfg.indicatorReduceTimerId =
      HEALTH_PANEL_REDUCE_INDICATOR_TIMER_ID;

  auto &energyPanelCfg = cfg.energyPanelCfg;
  energyPanelCfg.rsrcId = RoboCleanerGuiResources::ENERGY_PANEL;
  energyPanelCfg.indicatorRsrcId = RoboCleanerGuiResources::ENERGY_INDICATOR;
  energyPanelCfg.indicatorFontId = RoboCleanerGuiResources::VINQUE_RG_30;
  energyPanelCfg.indicatorReduceTimerId =
      ENERGY_PANEL_REDUCE_INDICATOR_TIMER_ID;

  return cfg;
}

FieldConfig generateFieldConfig() {
  FieldConfig cfg;

  const auto projectInstallPrefix =
      ament_index_cpp::get_package_share_directory(PROJECT_FOLDER_NAME);
  const auto levelId = 1;

  cfg.description =
      LevelFileLoader::readFieldDescription(projectInstallPrefix, levelId);
  cfg.tileRsrcId = RoboCleanerGuiResources::MAP_TILE;
  cfg.debugFontRsrcId = RoboCleanerGuiResources::VINQUE_RG_30;

  return cfg;
}

FogOfWarConfig generateFogOfWarConfig(int32_t mapTilesCount) {
  FogOfWarConfig cfg;
  cfg.status = FogOfWarStatus::ENABLED;
  cfg.cloudRsrcId = RoboCleanerGuiResources::FOG_OF_WAR;

  constexpr int startTimerId = FOG_OF_WAR_FADE_TIMER_IDS_START;
  cfg.fogTilesFadeAnimTimerIds.resize(mapTilesCount);
  std::iota(cfg.fogTilesFadeAnimTimerIds.begin(),
      cfg.fogTilesFadeAnimTimerIds.end(), startTimerId);

  return cfg;
}

EntityHandlerConfig generateEntityHandlerConfig() {
  EntityHandlerConfig cfg;
  cfg.rubbishRsrcId = RoboCleanerGuiResources::RUBBISH;
  cfg.rubbishFontId = RoboCleanerGuiResources::VINQUE_RG_30;
  cfg.obstacleRsrcId = RoboCleanerGuiResources::MAP_OBSTACLE;

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

  cfg.debugConsoleRsrcId = RoboCleanerGuiResources::VINQUE_RG_30;

  return cfg;
}

RoboCleanerGuiConfig generateGameConfig() {
  RoboCleanerGuiConfig cfg;
  auto& layoutCfg = cfg.layoutCfg;

  auto& commonLayoutCfg = layoutCfg.commonLayoutCfg;
  commonLayoutCfg.fieldCfg = generateFieldConfig();
  const auto mapTilesCount = commonLayoutCfg.fieldCfg.description.rows
      * commonLayoutCfg.fieldCfg.description.cols;

  commonLayoutCfg.fogOfWarConfig = generateFogOfWarConfig(mapTilesCount);
  commonLayoutCfg.robotBaseCfg = generateRobotBaseConfig();
  commonLayoutCfg.mapRsrcId = RoboCleanerGuiResources::MAP;
  commonLayoutCfg.playerFieldMarker = RoboCommonDefines::PLAYER_MARKER;

  layoutCfg.panelHandlerCfg = generatePanelHandlerConfig(mapTilesCount);
  layoutCfg.entityHandlerCfg = generateEntityHandlerConfig();

  return cfg;
}

} //end anonymous namespace

std::vector<DependencyDescription>
RoboCleanerGuiConfigGenerator::generateDependencies(
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

ApplicationConfig RoboCleanerGuiConfigGenerator::generateConfig() {
  ApplicationConfig cfg;
  cfg.engineCfg = generateEngineConfig();
  cfg.gameCfg = generateGameConfig();
  return cfg;
}

