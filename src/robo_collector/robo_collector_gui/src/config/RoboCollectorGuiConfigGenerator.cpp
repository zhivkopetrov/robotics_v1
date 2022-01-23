//Corresponding header
#include "robo_collector_gui/config/RoboCollectorGuiConfigGenerator.h"

//C system headers

//C++ system headers

//Other libraries headers
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "resource_utils/common/ResourceFileHeader.h"

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

enum TimerId {
  ROBOTS_ANIM_TIMER_ID_START,
  //reserved
  ROBOTS_ANIM_TIMER_ID_END = ROBOTS_ANIM_TIMER_ID_START + Defines::ENEMIES_CTN,

  COIN_PANEL_INCR_TIMER_ID,
  COIN_PANEL_DECR_TIMER_ID,

  COIN_ROTATE_ANIM_TIMER_ID_START,
  //reserved
  COIN_ROTATE_ANIM_TIMER_ID_END =
      COIN_ROTATE_ANIM_TIMER_ID_START + Defines::COINS_CTN,

  COIN_COLLECT_ANIM_TIMER_ID_START,
  //reserved
  COIN_COLLECT_ANIM_TIMER_ID_END =
      COIN_COLLECT_ANIM_TIMER_ID_START + Defines::COINS_CTN
};
}

EngineConfig RoboCollectorGuiConfigGenerator::generateEngineConfig() {
  const auto projectInstallPrefix =
      ament_index_cpp::get_package_share_directory(PROJECT_FOLDER_NAME);
  auto cfg = getDefaultEngineConfig(
      projectInstallPrefix, LOADING_SCREEN_RESOURCES_PATH);

  auto& windowCfg = cfg.managerHandlerCfg.drawMgrCfg.monitorWindowConfig;
  windowCfg.name = PROJECT_FOLDER_NAME;
  windowCfg.iconPath.append(projectInstallPrefix).append("/").
      append(ResourceFileHeader::getResourcesFolderName()).
      append("/p/entities/blinky.png");
  windowCfg.pos = Point(WINDOW_X, WINDOW_Y);
  windowCfg.width = WINDOW_WIDTH;
  windowCfg.height = WINDOW_HEIGHT;
  windowCfg.displayMode = WindowDisplayMode::WINDOWED;
  windowCfg.borderMode = WindowBorderMode::BORDERLESS;

  cfg.debugConsoleRsrcId = RoboCollectorGuiResources::VINQUE_RG_30;

  return cfg;
}

RoboCollectorGuiConfig RoboCollectorGuiConfigGenerator::generateGameConfig() {
  RoboCollectorGuiConfig cfg;

  cfg.mapRsrcId = RoboCollectorGuiResources::MAP;
  cfg.robotBlinkyRsrcId = RoboCollectorGuiResources::ROBO_BLINKY;
  cfg.robotEnemiesRsrcId = RoboCollectorGuiResources::ROBO_ENEMIES;
  cfg.robotsAnimStartTimerId = TimerId::ROBOTS_ANIM_TIMER_ID_START;

  cfg.coinAnimRsrcIds = {
      RoboCollectorGuiResources::COIN_ANIM_GOLD,
      RoboCollectorGuiResources::COIN_ANIM_SILVER,
      RoboCollectorGuiResources::COIN_ANIM_BRONZE
  };
  cfg.maxCoins = Defines::COINS_CTN;
  cfg.coinRotateAnimFirstTimerId = COIN_ROTATE_ANIM_TIMER_ID_START;
  cfg.coinCollectAnimFirstTimerId = COIN_COLLECT_ANIM_TIMER_ID_START;

  cfg.moveButtonsRsrcIds = {
      RoboCollectorGuiResources::UP_BUTTON,
      RoboCollectorGuiResources::LEFT_BUTTON,
      RoboCollectorGuiResources::RIGHT_BUTTON
  };
  cfg.maxMoveButtons = Defines::MOVE_BUTTONS_CTN;
  cfg.moveButtonsInfoTextFontId = RoboCollectorGuiResources::VINQUE_RG_30;

  constexpr auto GAME_FIELD_WIDTH = Defines::FIELD_COLS * Defines::TILE_WIDTH;
  constexpr auto GAME_FIELD_HEIGHT = Defines::FIELD_ROWS * Defines::TILE_HEIGHT;

  auto& fieldCfg = cfg.fieldCfg;
  fieldCfg.rows = Defines::FIELD_ROWS;
  fieldCfg.cols = Defines::FIELD_COLS;
  fieldCfg.fieldDimensions = { Defines::FIRST_TILE_X_POS,
      Defines::FIRST_TILE_Y_POS, GAME_FIELD_WIDTH, GAME_FIELD_HEIGHT };
  fieldCfg.tileWidth = Defines::TILE_WIDTH;
  fieldCfg.tileHeight = Defines::TILE_HEIGHT;
  fieldCfg.tileRsrcId = RoboCollectorGuiResources::MAP_TILE;
  fieldCfg.debugFontRsrcId = RoboCollectorGuiResources::VINQUE_RG_30;

  auto& panelCfg = cfg.panelConfig;
  panelCfg.timePanelRsrcId = RoboCollectorGuiResources::TIME_PANEL;
  panelCfg.healthPanelRsrcId = RoboCollectorGuiResources::HEALTH_PANEL;
  panelCfg.healthIndicatorRsrcId = RoboCollectorGuiResources::HEALTH_INDICATOR;
  panelCfg.horDelimiterRsrcId = RoboCollectorGuiResources::HOR_DELIMITER;
  panelCfg.vertDelimiterRsrcId = RoboCollectorGuiResources::VERT_DELIMITER;

  panelCfg.coinPanelRsrcId = RoboCollectorGuiResources::COIN_PANEL;
  panelCfg.coinPanelFontId = RoboCollectorGuiResources::VINQUE_RG_75;
  panelCfg.coinPanelIncrTimerId = COIN_PANEL_INCR_TIMER_ID;
  panelCfg.coinPanelDecrTimerId = COIN_PANEL_DECR_TIMER_ID;

  return cfg;
}

