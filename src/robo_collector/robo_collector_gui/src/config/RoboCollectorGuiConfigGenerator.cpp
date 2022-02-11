//Corresponding header
#include "robo_collector_gui/config/RoboCollectorGuiConfigGenerator.h"

//C system headers

//C++ system headers

//Other libraries headers
#include <rclcpp/utilities.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "resource_utils/common/ResourceFileHeader.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiDefines.h"
#include "generated/RoboCollectorGuiResources.h"

namespace {
//TODO parse the params from config
constexpr auto PROJECT_FOLDER_NAME = "robo_collector_gui";
constexpr auto LOADING_SCREEN_RESOURCES_PATH = "p/loading_screen/";

//screen
constexpr auto WINDOW_X = 72;
constexpr auto WINDOW_Y = 27;
constexpr auto WINDOW_WIDTH = 1848;
constexpr auto WINDOW_HEIGHT = 1053;

//misc
constexpr auto playerFieldMarker = 'B'; //B for Blinky
constexpr auto enemyFieldMarker = 'E'; //E for Enemy
constexpr auto emptyFieldMarker = '.';
constexpr auto totalGameSeconds = 180;

enum TimerId {
  ROBOTS_MOVE_ANIM_TIMER_ID_START,
  //reserved
  ROBOTS_MOVE_ANIM_TIMER_ID_END = ROBOTS_MOVE_ANIM_TIMER_ID_START
      + Defines::ENEMIES_CTN,

  ROBOTS_WALL_COLLISION_ANIM_TIMER_ID_START,
  //reserved
  ROBOTS_WALL_COLLISION_ANIM_TIMER_ID_END = ROBOTS_WALL_COLLISION_ANIM_TIMER_ID_START
      + Defines::ENEMIES_CTN,

  ROBOTS_ROBOT_COLLISION_ANIM_TIMER_ID_START,
  //reserved
  ROBOTS_ROBOT_COLLISION_ANIM_TIMER_ID_END = ROBOTS_ROBOT_COLLISION_ANIM_TIMER_ID_START
      + Defines::ENEMIES_CTN,

  ROBOTS_ROBOT_DAMAGE_ANIM_TIMER_ID_START,
  //reserved
  ROBOTS_ROBOT_DAMAGE_ANIM_TIMER_ID_END = ROBOTS_ROBOT_DAMAGE_ANIM_TIMER_ID_START
      + Defines::ENEMIES_CTN,

  COIN_ROTATE_ANIM_TIMER_ID_START,
  //reserved
  COIN_ROTATE_ANIM_TIMER_ID_END = COIN_ROTATE_ANIM_TIMER_ID_START
      + Defines::COINS_CTN,

  COIN_COLLECT_ANIM_TIMER_ID_START,
  //reserved
  COIN_COLLECT_ANIM_TIMER_ID_END = COIN_COLLECT_ANIM_TIMER_ID_START
      + Defines::COINS_CTN,

  COIN_RESPAWN_ANIM_TIMER_ID_START,
  //reserved
  COIN_RESPAWN_ANIM_TIMER_ID_END = COIN_RESPAWN_ANIM_TIMER_ID_START
      + Defines::COINS_CTN,

  COIN_PANEL_INCR_TIMER_ID,
  COIN_PANEL_DECR_TIMER_ID,
  TIME_PANEL_CLOCK_TIMER_ID,
  TIME_PANEL_BLINK_TIMER_ID,
  HEALTH_PANEL_REDUCE_INDICATOR_TIMER_ID
};

RobotBaseConfig generateRobotBaseConfig() {
  RobotBaseConfig cfg;

  cfg.playerRsrcId = RoboCollectorGuiResources::PLAYER_ROBOT;
  cfg.enemiesRsrcId = RoboCollectorGuiResources::ENEMY_ROBOTS;
  cfg.damageMarkerRsrcId = RoboCollectorGuiResources::DAMAGE_MARKER;
  cfg.moveAnimStartTimerId = ROBOTS_MOVE_ANIM_TIMER_ID_START;
  cfg.wallCollisionAnimStartTimerId = ROBOTS_WALL_COLLISION_ANIM_TIMER_ID_START;
  cfg.robotCollisionAnimStartTimerId =
      ROBOTS_ROBOT_COLLISION_ANIM_TIMER_ID_START;
  cfg.robotDamageAnimStartTimerId = ROBOTS_ROBOT_DAMAGE_ANIM_TIMER_ID_START;

  return cfg;
}

RoboCollectorControllerConfig generateRoboCollectorControllerConfig() {
  RoboCollectorControllerConfig cfg;

  cfg.moveButtonsRsrcIds = { RoboCollectorGuiResources::UP_BUTTON,
      RoboCollectorGuiResources::LEFT_BUTTON,
      RoboCollectorGuiResources::RIGHT_BUTTON };
  cfg.maxMoveButtons = Defines::MOVE_BUTTONS_CTN;
  cfg.moveButtonInfoTextFontId = RoboCollectorGuiResources::VINQUE_RG_30;
  cfg.horDelimiterRsrcId = RoboCollectorGuiResources::HOR_DELIMITER;
  cfg.vertDelimiterRsrcId = RoboCollectorGuiResources::VERT_DELIMITER;
  cfg.helpButtonRsrcId = RoboCollectorGuiResources::HELP_BUTTON;
  cfg.settingsButtonRsrcId = RoboCollectorGuiResources::SETTINGS_BUTTON;

  return cfg;
}

CoinHandlerConfig generateCoinHandlerConfig() {
  CoinHandlerConfig cfg;

  cfg.animRsrcIds = { RoboCollectorGuiResources::COIN_ANIM_GOLD,
      RoboCollectorGuiResources::COIN_ANIM_SILVER,
      RoboCollectorGuiResources::COIN_ANIM_BRONZE };
  cfg.fieldMarkers = { 'g', 's', 'b' //gold, silver, bronze
      };
  cfg.maxCoins = Defines::COINS_CTN;
  cfg.rotateAnimFirstTimerId = COIN_ROTATE_ANIM_TIMER_ID_START;
  cfg.collectAnimFirstTimerId = COIN_COLLECT_ANIM_TIMER_ID_START;
  cfg.respawnAnimFirstTimerId = COIN_RESPAWN_ANIM_TIMER_ID_START;
  cfg.fieldEmptyMarker = emptyFieldMarker;

  return cfg;
}

PanelHandlerConfig generatePanelHandlerConfig() {
  PanelHandlerConfig cfg;

  auto &healthPanelCfg = cfg.healthPanelCfg;
  healthPanelCfg.rsrcId = RoboCollectorGuiResources::HEALTH_PANEL;
  healthPanelCfg.indicatorRsrcId = RoboCollectorGuiResources::HEALTH_INDICATOR;
  healthPanelCfg.indicatorFontId = RoboCollectorGuiResources::VINQUE_RG_30;
  healthPanelCfg.indicatorReduceTimerId =
      HEALTH_PANEL_REDUCE_INDICATOR_TIMER_ID;

  auto &coinPanelCfg = cfg.coinPanelCfg;
  coinPanelCfg.rsrcId = RoboCollectorGuiResources::COIN_PANEL;
  coinPanelCfg.fontId = RoboCollectorGuiResources::VINQUE_RG_75;
  coinPanelCfg.incrTimerId = COIN_PANEL_INCR_TIMER_ID;
  coinPanelCfg.decrTimerId = COIN_PANEL_DECR_TIMER_ID;

  auto &timePanelCfg = cfg.timePanelCfg;
  timePanelCfg.rsrcId = RoboCollectorGuiResources::TIME_PANEL;
  timePanelCfg.fontId = RoboCollectorGuiResources::VINQUE_RG_75;
  timePanelCfg.clockTimerId = TIME_PANEL_CLOCK_TIMER_ID;
  timePanelCfg.blinkTimerId = TIME_PANEL_BLINK_TIMER_ID;
  timePanelCfg.totalSeconds = totalGameSeconds;

  return cfg;
}

FieldConfig generateFieldConfig() {
  FieldConfig cfg;
  constexpr auto GAME_FIELD_WIDTH = Defines::FIELD_COLS * Defines::TILE_WIDTH;
  constexpr auto GAME_FIELD_HEIGHT = Defines::FIELD_ROWS * Defines::TILE_HEIGHT;

  cfg.rows = Defines::FIELD_ROWS;
  cfg.cols = Defines::FIELD_COLS;
  cfg.fieldDimensions = { Defines::FIRST_TILE_X_POS, Defines::FIRST_TILE_Y_POS,
      GAME_FIELD_WIDTH, GAME_FIELD_HEIGHT };
  cfg.tileWidth = Defines::TILE_WIDTH;
  cfg.tileHeight = Defines::TILE_HEIGHT;
  cfg.tileRsrcId = RoboCollectorGuiResources::MAP_TILE;
  cfg.debugFontRsrcId = RoboCollectorGuiResources::VINQUE_RG_30;

  return cfg;
}

EngineConfig generateEngineConfig() {
  const auto projectInstallPrefix =
      ament_index_cpp::get_package_share_directory(PROJECT_FOLDER_NAME);
  auto cfg = getDefaultEngineConfig(projectInstallPrefix,
      LOADING_SCREEN_RESOURCES_PATH);

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

  cfg.debugConsoleRsrcId = RoboCollectorGuiResources::VINQUE_RG_30;

  return cfg;
}

RoboMinerGuiConfig generateRoboMinerGuiConfig() {
  RoboMinerGuiConfig cfg;
  cfg.crystalRsrcId = RoboCollectorGuiResources::CRYSTALS;

  auto& fieldCfg = cfg.fieldCfg;
  fieldCfg.emptyTileMarker = emptyFieldMarker;
  fieldCfg.rows = Defines::FIELD_ROWS;
  fieldCfg.cols = Defines::FIELD_COLS;

  return cfg;
}

RoboCleanerGuiConfig generateRoboCleanerGuiConfig() {
  RoboCleanerGuiConfig cfg;
  cfg.rubbishRsrcId = RoboCollectorGuiResources::RUBBISH;
  cfg.rubbishFontId = RoboCollectorGuiResources::VINQUE_RG_30;
  cfg.obstacleRsrcId = RoboCollectorGuiResources::MAP_OBSTACLE;

  auto& energyPanelCfg = cfg.energyPanelCfg;
  energyPanelCfg.rsrcId = RoboCollectorGuiResources::ENERGY_PANEL;
  energyPanelCfg.indicatorRsrcId = RoboCollectorGuiResources::ENERGY_INDICATOR;

  return cfg;
}

RoboCollectorGuiConfig generateGameConfig() {
  RoboCollectorGuiConfig cfg;
  cfg.fieldCfg = generateFieldConfig();
  cfg.panelHandlerCfg = generatePanelHandlerConfig();
  cfg.robotBaseCfg = generateRobotBaseConfig();
  cfg.coinHandlerCfg = generateCoinHandlerConfig();
  cfg.controllerCfg = generateRoboCollectorControllerConfig();

  cfg.mapRsrcId = RoboCollectorGuiResources::MAP;
  cfg.playerFieldMarker = playerFieldMarker;
  cfg.enemyFieldMarker = enemyFieldMarker;

  cfg.roboMinerGuiConfig = generateRoboMinerGuiConfig();
  cfg.roboCleanerGuiConfig = generateRoboCleanerGuiConfig();

  return cfg;
}

} //end anonymous namespace

std::vector<DependencyDescription>
RoboCollectorGuiConfigGenerator::generateDependencies(
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

ApplicationConfig RoboCollectorGuiConfigGenerator::generateConfig() {
  ApplicationConfig cfg;
  cfg.engineCfg = generateEngineConfig();
  cfg.gameCfg = generateGameConfig();
  return cfg;
}

