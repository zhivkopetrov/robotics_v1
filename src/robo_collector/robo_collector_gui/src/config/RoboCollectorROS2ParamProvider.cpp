//Corresponding header
#include "robo_collector_gui/config/RoboCollectorROS2ParamProvider.h"

//System headers
#include <sstream>

//Other libraries headers
#include "utils/Log.h"

//Own components headers

namespace {
constexpr auto GUI_WINDOW_X_PARAM_NAME = "gui_window_x";
constexpr auto GUI_WINDOW_Y_PARAM_NAME = "gui_window_y";
constexpr auto GUI_WINDOW_WIDTH_PARAM_NAME = "gui_window_width";
constexpr auto GUI_WINDOW_HEIGHT_PARAM_NAME = "gui_window_height";
constexpr auto TOTAL_GAME_SECONDS_PARAM_NAME = "total_game_seconds";
constexpr auto TARGET_WIN_COINS_PARAM_NAME = "target_win_coins";
constexpr auto USE_LOCAL_CONTROLLER_MODE_PARAM_NAME =
    "use_local_controller_mode";

//screen
constexpr auto DEFAULT_WINDOW_X = 72;
constexpr auto DEFAULT_WINDOW_Y = 27;
constexpr auto DEFAULT_WINDOW_WIDTH = 1848;
constexpr auto DEFAULT_WINDOW_HEIGHT = 1053;

//misc
constexpr auto DEFAULT_TOTAL_GAME_SECONDS = 180;
constexpr auto DEFAULT_USE_LOCAL_CONTROLLER_MODE = false;
constexpr auto DEFAULT_TARGET_WIN_COINS = 30;
}

void RoboCollectorGuiRos2Params::print() const {
  std::ostringstream ostr;
  ostr << "==================================================================\n"
       << "Printing RoboCollectorGuiRos2Params:\n"
       << GUI_WINDOW_X_PARAM_NAME << ": " << guiWindow.x << '\n'
       << GUI_WINDOW_Y_PARAM_NAME << ": " << guiWindow.y << '\n'
       << GUI_WINDOW_WIDTH_PARAM_NAME << ": " << guiWindow.w << '\n'
       << GUI_WINDOW_HEIGHT_PARAM_NAME << ": " << guiWindow.h << '\n'
       << TOTAL_GAME_SECONDS_PARAM_NAME << ": " << totalGameSeconds << '\n'
       << TARGET_WIN_COINS_PARAM_NAME << ": " << targetWinCoins << '\n'
       << USE_LOCAL_CONTROLLER_MODE_PARAM_NAME << ": "
           << ((LocalControllerMode::ENABLED == localControrllerMode) ?
               "true" : "false") << '\n'
       << "=================================================================\n";

  LOG("%s", ostr.str().c_str());
}

RoboCollectorROS2ParamProvider::RoboCollectorROS2ParamProvider()
    : rclcpp::Node("RoboCollectorGuiRos2ParamProvider") {
  declare_parameter<int32_t>(GUI_WINDOW_X_PARAM_NAME, DEFAULT_WINDOW_X);
  declare_parameter<int32_t>(GUI_WINDOW_Y_PARAM_NAME, DEFAULT_WINDOW_Y);
  declare_parameter<int32_t>(GUI_WINDOW_WIDTH_PARAM_NAME, DEFAULT_WINDOW_WIDTH);
  declare_parameter<int32_t>(GUI_WINDOW_HEIGHT_PARAM_NAME,
      DEFAULT_WINDOW_HEIGHT);
  declare_parameter<int32_t>(TOTAL_GAME_SECONDS_PARAM_NAME,
      DEFAULT_TOTAL_GAME_SECONDS);
  declare_parameter<int32_t>(TARGET_WIN_COINS_PARAM_NAME,
      DEFAULT_TARGET_WIN_COINS);
  declare_parameter<bool>(USE_LOCAL_CONTROLLER_MODE_PARAM_NAME,
      DEFAULT_USE_LOCAL_CONTROLLER_MODE);
}

RoboCollectorGuiRos2Params RoboCollectorROS2ParamProvider::getParams() {
  get_parameter(GUI_WINDOW_X_PARAM_NAME, _params.guiWindow.x);
  get_parameter(GUI_WINDOW_Y_PARAM_NAME, _params.guiWindow.y);
  get_parameter(GUI_WINDOW_WIDTH_PARAM_NAME, _params.guiWindow.w);
  get_parameter(GUI_WINDOW_HEIGHT_PARAM_NAME, _params.guiWindow.h);

  get_parameter(TOTAL_GAME_SECONDS_PARAM_NAME, _params.totalGameSeconds);
  get_parameter(TARGET_WIN_COINS_PARAM_NAME, _params.targetWinCoins);

  bool useLocalControllerMode{};
  get_parameter(USE_LOCAL_CONTROLLER_MODE_PARAM_NAME, useLocalControllerMode);
  _params.localControrllerMode = useLocalControllerMode ?
      LocalControllerMode::ENABLED : LocalControllerMode::DISABLED;

  return _params;
}
