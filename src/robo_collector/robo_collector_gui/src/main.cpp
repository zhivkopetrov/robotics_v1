//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "game_engine/Application.h"
#include "resource_utils/common/ResourceFileHeader.h"
#include "utils/file_system/FileSystemUtils.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/RoboCollectorGui.h"
#include "robo_collector_gui/config/RoboCollectorGuiConfig.h"
#include "generated/RoboCollectorGuiResources.h"

namespace {
//TODO parse the params from config
constexpr auto PROJECT_FOLDER_NAME = "robo_collector_gui";
constexpr auto LOADING_SCREEN_RESOURCES_PATH = "p/loading_screen/";

//game field tiles
constexpr auto TILE_WIDTH = 160;
constexpr auto TILE_HEIGHT = 160;

//game field
constexpr auto GAME_MODE = GameMode::NORMAL;
constexpr auto GAME_FIELD_ROWS = 5;
constexpr auto GAME_FIELD_COLS = 7;
constexpr auto GAME_FIELD_START_X = 150;
constexpr auto GAME_FIELD_START_Y = 150;
//NOTE: (TILE_WIDTH / 2) and (TILE_HEIGHT / 2) guarantee the tile offset
constexpr auto GAME_FIELD_WIDTH =
    (GAME_FIELD_COLS * TILE_WIDTH) + (TILE_WIDTH / 2);
constexpr auto GAME_FIELD_HEIGHT =
    (GAME_FIELD_ROWS * TILE_HEIGHT) + (TILE_HEIGHT / 2);
}

static EngineConfig populateEngineConfig() {
  const auto projectInstallPrefix =
      ament_index_cpp::get_package_share_directory(PROJECT_FOLDER_NAME);
  auto cfg = getDefaultEngineConfig(
      projectInstallPrefix, LOADING_SCREEN_RESOURCES_PATH);

  cfg.debugConsoleRsrcId = RoboCollectorGuiResources::VINQUE_RG;

  return cfg;
}

static RoboCollectorGuiConfig populateGuiConfig() {
  RoboCollectorGuiConfig cfg;

  cfg.gameMode = GAME_MODE;
  cfg.fieldCfg.rows = GAME_FIELD_ROWS;
  cfg.fieldCfg.cols = GAME_FIELD_COLS;
  cfg.fieldCfg.fieldDimensions = { GAME_FIELD_START_X,
      GAME_FIELD_START_Y, GAME_FIELD_WIDTH, GAME_FIELD_HEIGHT };
  cfg.fieldCfg.tileWidth = TILE_WIDTH;
  cfg.fieldCfg.tileHeight = TILE_HEIGHT;
  cfg.fieldCfg.tileRsrcId = RoboCollectorGuiResources::MAP_TILE;
  cfg.fieldCfg.debugFontRsrcId = RoboCollectorGuiResources::VINQUE_RG;

  return cfg;
}

int32_t main(int32_t argc, char *args[]) {
  rclcpp::init(argc, args);

  const auto engineCfg = populateEngineConfig();
  const auto guiCfg = populateGuiConfig();

  std::unique_ptr<Game> game = std::make_unique<RoboCollectorGui>();
  Application app(std::move(game));
  if (SUCCESS != app.init(engineCfg, guiCfg)) {
    LOGERR("app.init() failed");
    return FAILURE;
  }

  return app.run();
}



//#include <chrono>
//
//#include "rclcpp/rclcpp.hpp"
//#include "robo_collector_interfaces/msg/direction.hpp"
//#include <iostream>
//
//using namespace std::chrono_literals;
//
///* This example creates a subclass of Node and uses a fancy C++11 lambda
// * function to shorten the callback syntax, at the expense of making the
// * code somewhat more difficult to understand at first glance. */
//
//class MinimalPublisher : public rclcpp::Node
//{
//public:
//  MinimalPublisher()
//  : Node("minimal_publisher"), count_(0)
//  {
//    publisher_ = create_publisher<robo_collector_interfaces::msg::Direction>("direction", 10);
//    auto timer_callback =
//      [this]() -> void {
//    	robo_collector_interfaces::msg::Direction msg;
//    	msg.direction = robo_collector_interfaces::msg::Direction::DIRECTION_UP;
//        RCLCPP_INFO(this->get_logger(),
//        		"Publishing dir: '%hhu'", msg.direction);
//        publisher_->publish(msg);
//      };
//    timer_ = this->create_wall_timer(1s, timer_callback);
//  }
//
//private:
//  rclcpp::TimerBase::SharedPtr timer_;
//  rclcpp::Publisher<robo_collector_interfaces::msg::Direction>::SharedPtr publisher_;
//  size_t count_;
//};
//
//#include <iostream>
//
//int main(int argc, char * argv[])
//{
//  rclcpp::init(argc, argv);
//  rclcpp::spin(std::make_shared<MinimalPublisher>());
//  rclcpp::shutdown();
//  return 0;
//}
