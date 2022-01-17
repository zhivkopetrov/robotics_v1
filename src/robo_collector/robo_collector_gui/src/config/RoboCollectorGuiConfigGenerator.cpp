//Corresponding header
#include "robo_collector_gui/config/RoboCollectorGuiConfigGenerator.h"

//C system headers

//C++ system headers

//Other libraries headers
#include <ament_index_cpp/get_package_share_directory.hpp>

//Own components headers
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

//game field tiles
constexpr auto TILE_WIDTH = 160;
constexpr auto TILE_HEIGHT = 160;

//game field
constexpr auto GAME_MODE = GameMode::NORMAL;
constexpr auto GAME_FIELD_ROWS = 6;
constexpr auto GAME_FIELD_COLS = 7;
constexpr auto GAME_FIELD_START_X = 47;
constexpr auto GAME_FIELD_START_Y = 47;
constexpr auto GAME_FIELD_WIDTH = GAME_FIELD_COLS * TILE_WIDTH;
constexpr auto GAME_FIELD_HEIGHT = GAME_FIELD_ROWS * TILE_HEIGHT;
}

EngineConfig RoboCollectorGuiConfigGenerator::generateEngineConfig() {
  const auto projectInstallPrefix =
      ament_index_cpp::get_package_share_directory(PROJECT_FOLDER_NAME);
  auto cfg = getDefaultEngineConfig(
      projectInstallPrefix, LOADING_SCREEN_RESOURCES_PATH);

  auto& windowCfg = cfg.managerHandlerCfg.drawMgrCfg.monitorWindowConfig;
  windowCfg.name = PROJECT_FOLDER_NAME;
  windowCfg.pos = Point(WINDOW_X, WINDOW_Y);
  windowCfg.width = WINDOW_WIDTH;
  windowCfg.height = WINDOW_HEIGHT;
  windowCfg.displayMode = WindowDisplayMode::WINDOWED;
  windowCfg.borderMode = WindowBorderMode::BORDERLESS;

  cfg.debugConsoleRsrcId = RoboCollectorGuiResources::VINQUE_RG;

  return cfg;
}

RoboCollectorGuiConfig RoboCollectorGuiConfigGenerator::generateGameConfig() {
  RoboCollectorGuiConfig cfg;

  cfg.gameMode = GAME_MODE;
  cfg.robotBlinkyRsrcId = RoboCollectorGuiResources::ROBO_BLINKY;
  cfg.robotEnemiesRsrcId = RoboCollectorGuiResources::ROBO_ENEMIES;

  auto& fieldCfg = cfg.fieldCfg;
  fieldCfg.rows = GAME_FIELD_ROWS;
  fieldCfg.cols = GAME_FIELD_COLS;
  fieldCfg.fieldDimensions = { GAME_FIELD_START_X,
      GAME_FIELD_START_Y, GAME_FIELD_WIDTH, GAME_FIELD_HEIGHT };
  fieldCfg.tileWidth = TILE_WIDTH;
  fieldCfg.tileHeight = TILE_HEIGHT;
  fieldCfg.tileRsrcId = RoboCollectorGuiResources::MAP_TILE;
  fieldCfg.mapRsrcId = RoboCollectorGuiResources::MAP;
  fieldCfg.debugFontRsrcId = RoboCollectorGuiResources::VINQUE_RG;

  auto& panelCfg = cfg.panelConfig;
  panelCfg.timePanelRsrcId = RoboCollectorGuiResources::TIME_PANEL;
  panelCfg.coinPanelRsrcId = RoboCollectorGuiResources::COIN_PANEL;
  panelCfg.healthPanelRsrcId = RoboCollectorGuiResources::HEALTH_PANEL;
  panelCfg.healthIndicatorRsrcId = RoboCollectorGuiResources::HEALTH_INDICATOR;

  return cfg;
}

