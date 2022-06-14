//Corresponding header
#include "robo_collector_gui/config/RoboCollectorGuiConfigGenerator.h"

//System headers

//Other libraries headers
#include <rclcpp/utilities.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "robo_common/defines/RoboCommonDefines.h"
#include "robo_common/helpers/LevelFileLoader.h"
#include "resource_utils/common/ResourceFileHeader.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/config/RoboCollectorGuiConfig.h"
#include "robo_collector_gui/config/RoboCollectorROS2ParamProvider.h"
#include "robo_collector_gui/defines/RoboCollectorGuiDefines.h"
#include "generated/RoboCollectorGuiResources.h"

namespace {
constexpr auto PROJECT_FOLDER_NAME = "robo_collector_gui";
constexpr auto ROBOT_FIELD_MARKERS = RobotFieldMarkers::ENABLED;

enum TimerId {
  ROBOTS_MOVE_ANIM_TIMER_ID_START,
  //reserved
  ROBOTS_MOVE_ANIM_TIMER_ID_END = ROBOTS_MOVE_ANIM_TIMER_ID_START
      + Defines::ENEMIES_CTN,

  ROBOTS_ROTATE_ANIM_TIMER_ID_START,
  //reserved
  ROBOTS_ROTATE_ANIM_TIMER_ID_END = ROBOTS_ROTATE_ANIM_TIMER_ID_START
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
  HEALTH_PANEL_MODIFY_INDICATOR_TIMER_ID
};

RobotBaseConfig generateRobotBaseConfig() {
  RobotBaseConfig cfg;

  cfg.playerRsrcId = RoboCollectorGuiResources::PLAYER_ROBOT;
  cfg.enemiesRsrcId = RoboCollectorGuiResources::ENEMY_ROBOTS;
  cfg.damageMarkerRsrcId = RoboCollectorGuiResources::DAMAGE_MARKER;
  cfg.moveAnimStartTimerId = ROBOTS_MOVE_ANIM_TIMER_ID_START;
  cfg.rotateAnimStartTimerId = ROBOTS_ROTATE_ANIM_TIMER_ID_START;
  cfg.robotCollisionAnimStartTimerId =
      ROBOTS_ROBOT_COLLISION_ANIM_TIMER_ID_START;
  cfg.robotDamageAnimStartTimerId = ROBOTS_ROBOT_DAMAGE_ANIM_TIMER_ID_START;
  cfg.robotFieldMarkers = ROBOT_FIELD_MARKERS;

  return cfg;
}

RoboCollectorUiControllerBaseConfig generateRoboCollectorUiControllerConfig(
    LocalControllerMode localControllerMode) {
  RoboCollectorUiControllerBaseConfig cfg;

  cfg.moveButtonsRsrcIds = { RoboCollectorGuiResources::UP_BUTTON,
      RoboCollectorGuiResources::LEFT_BUTTON,
      RoboCollectorGuiResources::RIGHT_BUTTON };
  cfg.moveButtonInfoTextFontId = RoboCollectorGuiResources::VINQUE_RG_30;
  cfg.horDelimiterRsrcId = RoboCollectorGuiResources::HOR_DELIMITER;
  cfg.vertDelimiterRsrcId = RoboCollectorGuiResources::VERT_DELIMITER;
  cfg.helpButtonRsrcId = RoboCollectorGuiResources::HELP_BUTTON;
  cfg.settingsButtonRsrcId = RoboCollectorGuiResources::SETTINGS_BUTTON;
  cfg.localControllerMode = localControllerMode;

  return cfg;
}

CoinHandlerConfig generateCoinHandlerConfig(int32_t targetWinCoints) {
  CoinHandlerConfig cfg;

  cfg.animRsrcIds = { RoboCollectorGuiResources::COIN_ANIM_GOLD,
      RoboCollectorGuiResources::COIN_ANIM_SILVER,
      RoboCollectorGuiResources::COIN_ANIM_BRONZE };
  cfg.fieldMarkers = { 'g', 's', 'b' //gold, silver, bronze
      };
  cfg.maxCoins = Defines::COINS_CTN;
  cfg.targetWinCoins = targetWinCoints;
  cfg.rotateAnimFirstTimerId = COIN_ROTATE_ANIM_TIMER_ID_START;
  cfg.collectAnimFirstTimerId = COIN_COLLECT_ANIM_TIMER_ID_START;
  cfg.respawnAnimFirstTimerId = COIN_RESPAWN_ANIM_TIMER_ID_START;
  cfg.fieldEmptyMarker = RoboCommonDefines::EMPTY_TILE_MARKER;

  return cfg;
}

PanelHandlerConfig generatePanelHandlerConfig(int32_t targetWinCoints,
                                              int32_t totalGameSeconds) {
  PanelHandlerConfig cfg;

  auto &healthPanelCfg = cfg.healthPanelCfg;
  healthPanelCfg.rsrcId = RoboCollectorGuiResources::HEALTH_PANEL;
  healthPanelCfg.indicatorRsrcId = RoboCollectorGuiResources::HEALTH_INDICATOR;
  healthPanelCfg.indicatorFontId = RoboCollectorGuiResources::VINQUE_RG_30;
  healthPanelCfg.indicatorModifyTimerId =
      HEALTH_PANEL_MODIFY_INDICATOR_TIMER_ID;

  auto &coinPanelCfg = cfg.coinPanelCfg;
  coinPanelCfg.targetNumber = targetWinCoints;
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

ObstacleHandlerConfig generateObstacleHandlerConfig() {
  ObstacleHandlerConfig cfg;

  cfg.obstacleRsrcId = RoboCollectorGuiResources::MAP_OBSTACLE;
  cfg.status = ObstacleHandlerApproachOverlayStatus::DISABLED;

  return cfg;
}

FieldConfig generateFieldConfig(const FieldDescription& fieldDescr) {
  FieldConfig cfg;

  cfg.description = fieldDescr;
  cfg.obstacleHandlerConfig = generateObstacleHandlerConfig();
  cfg.tileRsrcId = RoboCollectorGuiResources::MAP_TILE;
  cfg.debugFontRsrcId = RoboCollectorGuiResources::VINQUE_RG_30;

  return cfg;
}

EngineConfig generateEngineConfig(const std::string& projectInstallPrefix,
                                  const RoboCollectorGuiRos2Params& rosParams) {
  auto cfg = getDefaultEngineConfig(projectInstallPrefix);

  auto &windowCfg = cfg.managerHandlerCfg.drawMgrCfg.monitorWindowConfig;
  windowCfg.name = PROJECT_FOLDER_NAME;
  windowCfg.iconPath.append(projectInstallPrefix).append("/").append(
      ResourceFileHeader::getResourcesFolderName()).append(
      "/p/entities/player_robot.png");
  windowCfg.pos = Point(rosParams.guiWindow.x, rosParams.guiWindow.y);
  windowCfg.width = rosParams.guiWindow.w;
  windowCfg.height = rosParams.guiWindow.h;
  windowCfg.displayMode = WindowDisplayMode::WINDOWED;
  windowCfg.borderMode = WindowBorderMode::BORDERLESS;

  cfg.debugConsoleConfig.fontRsrcId = RoboCollectorGuiResources::VINQUE_RG_30;

  return cfg;
}

RoboCollectorGuiConfig generateGameConfig(
    const std::string& projectInstallPrefix,
    const RoboCollectorGuiRos2Params& rosParams) {
  RoboCollectorGuiConfig cfg;
  const auto levelId = 1;
  const auto [fieldDescr, initialRobotState] =
      LevelFileLoader::readLevelData(projectInstallPrefix, levelId);

  auto &layoutCfg = cfg.layoutCfg;
  layoutCfg.panelHandlerCfg = generatePanelHandlerConfig(
      rosParams.targetWinCoins, rosParams.totalGameSeconds);
  layoutCfg.coinHandlerCfg = generateCoinHandlerConfig(rosParams.targetWinCoins);
  layoutCfg.controllerCfg = generateRoboCollectorUiControllerConfig(
      rosParams.localControrllerMode);

  auto &commonLayoutCfg = layoutCfg.commonLayoutCfg;
  commonLayoutCfg.fieldCfg = generateFieldConfig(fieldDescr);
  commonLayoutCfg.robotInitialState = initialRobotState;
  commonLayoutCfg.robotBaseCfg = generateRobotBaseConfig();
  commonLayoutCfg.mapRsrcId = RoboCollectorGuiResources::MAP;
  commonLayoutCfg.playerFieldMarker = RoboCommonDefines::PLAYER_MARKER;
  commonLayoutCfg.enemyFieldMarker = RoboCommonDefines::ENEMY_MARKER;

  return cfg;
}

} //end anonymous namespace

std::vector<DependencyDescription>
RoboCollectorGuiConfigGenerator::generateDependencies(
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

ApplicationConfig RoboCollectorGuiConfigGenerator::generateConfig() {
  ApplicationConfig cfg;
  const auto projectInstallPrefix =
      ament_index_cpp::get_package_share_directory(PROJECT_FOLDER_NAME);

  auto paramProviderNode = std::make_shared<RoboCollectorROS2ParamProvider>();
  const auto rosParams = paramProviderNode->getParams();
  rosParams.print();

  cfg.engineCfg = generateEngineConfig(projectInstallPrefix, rosParams);
  cfg.gameCfg = generateGameConfig(projectInstallPrefix, rosParams);

  return cfg;
}

