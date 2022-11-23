//Corresponding header
#include "robo_collector_gui/external_api/RoboCollectorGuiRos2ParamProvider.h"

//System headers
#include <sstream>
#include <thread>

//Other libraries headers
#include "utils/data_type/EnumClassUtils.h"
#include "utils/Log.h"

//Own components headers

namespace {
constexpr auto NODE_NAME = "RoboCollectorGuiRos2ParamProvider";

constexpr auto GUI_WINDOW_X_PARAM_NAME = "gui_window_x";
constexpr auto GUI_WINDOW_Y_PARAM_NAME = "gui_window_y";
constexpr auto GUI_WINDOW_WIDTH_PARAM_NAME = "gui_window_width";
constexpr auto GUI_WINDOW_HEIGHT_PARAM_NAME = "gui_window_height";

constexpr auto ENGINE_TARGET_FPS_PARAM_NAME = "engine_target_fps";
constexpr auto RENDERER_FLAGS_MASK_PARAM_NAME = "renderer_flags_mask";
constexpr auto FBO_OPTIMIZATIONS_ENABLED_PARAM_NAME =
    "fbo_optimizations_enabled";

constexpr auto ROS2_EXECUTOR_TYPE_PARAM_NAME = "ros2_executor_type";
constexpr auto ROS2_EXECUTOR_THREADS_NUM_PARAM_NAME = "ros2_executor_threads_num";

constexpr auto TOTAL_GAME_SECONDS_PARAM_NAME = "total_game_seconds";
constexpr auto TARGET_WIN_COINS_PARAM_NAME = "target_win_coins";
constexpr auto USE_LOCAL_CONTROLLER_MODE_PARAM_NAME =
    "use_local_controller_mode";

//screen
constexpr auto DEFAULT_WINDOW_X = 72;
constexpr auto DEFAULT_WINDOW_Y = 27;
constexpr auto DEFAULT_WINDOW_WIDTH = 1848;
constexpr auto DEFAULT_WINDOW_HEIGHT = 1053;

//Renderer
constexpr auto DEFAULT_RENDERER_FLAGS_MASK =
    getEnumValue(RendererFlag::HARDWARE_RENDERER) |
    getEnumValue(RendererFlag::FBO_ENABLE);
constexpr auto DEFAULT_ENGINE_TARGET_FPS = 60u;
constexpr auto DEFAULT_FBO_OPTIMIZATIONS_ENABLED = true;

//ROS2 executor
constexpr auto DEFAULT_EXECUTOR_TYPE = 0;
constexpr auto DEFAULT_EXECUTOR_THREADS_NUM = 2;

//misc
constexpr auto DEFAULT_TOTAL_GAME_SECONDS = 180;
constexpr auto DEFAULT_USE_LOCAL_CONTROLLER_MODE = false;
constexpr auto DEFAULT_TARGET_WIN_COINS = 30;

template<typename T>
void handleParamError(const char* paramName, T& value, const T& defaultValue) {
  std::ostringstream ostr;
  ostr << "Param: [" << paramName << "] has invalid value: [" << value
       << "]. Overriding with default value: [" << defaultValue << "]";
  LOGR("%s", ostr.str().c_str());

  value = defaultValue;
}
}

void RoboCollectorGuiRos2Params::print() const {
  std::ostringstream ostr;
  ostr << "==================================================================\n"
      << "Printing node(" << NODE_NAME << ") params:\n"
       << GUI_WINDOW_X_PARAM_NAME << ": " << guiWindow.x << '\n'
       << GUI_WINDOW_Y_PARAM_NAME << ": " << guiWindow.y << '\n'
       << GUI_WINDOW_WIDTH_PARAM_NAME << ": " << guiWindow.w << '\n'
       << GUI_WINDOW_HEIGHT_PARAM_NAME << ": " << guiWindow.h << '\n'
       << ENGINE_TARGET_FPS_PARAM_NAME << ": " << engineTargetFps << '\n'
       << RENDERER_FLAGS_MASK_PARAM_NAME << ": " << rendererFlagsMask << '\n'
       << FBO_OPTIMIZATIONS_ENABLED_PARAM_NAME << ": "
           << ((FboOptimization::ENABLED == fboOptimization) ?
               "true" : "false") << '\n'
       << ROS2_EXECUTOR_TYPE_PARAM_NAME << ": " <<
         getExecutorName(ros2CommunicatorConfig.executorType) << '\n'
       << ROS2_EXECUTOR_THREADS_NUM_PARAM_NAME << ": "
         << ros2CommunicatorConfig.numberOfThreads << '\n'
       << TOTAL_GAME_SECONDS_PARAM_NAME << ": " << totalGameSeconds << '\n'
       << TARGET_WIN_COINS_PARAM_NAME << ": " << targetWinCoins << '\n'
       << USE_LOCAL_CONTROLLER_MODE_PARAM_NAME << ": "
           << ((LocalControllerMode::ENABLED == localControrllerMode) ?
               "true" : "false") << '\n'
       << "=================================================================\n";

  LOG("%s", ostr.str().c_str());
}

void RoboCollectorGuiRos2Params::validate() {
  if (0 >= guiWindow.w) {
    handleParamError(GUI_WINDOW_WIDTH_PARAM_NAME, guiWindow.w,
        DEFAULT_WINDOW_WIDTH);
  }
  if (0 >= guiWindow.h) {
    handleParamError(GUI_WINDOW_HEIGHT_PARAM_NAME, guiWindow.h,
        DEFAULT_WINDOW_HEIGHT);
  }
  if (0 >= totalGameSeconds) {
    handleParamError(TOTAL_GAME_SECONDS_PARAM_NAME, totalGameSeconds,
        DEFAULT_TOTAL_GAME_SECONDS);
  }
  if (0 >= targetWinCoins) {
    handleParamError(TARGET_WIN_COINS_PARAM_NAME, targetWinCoins,
        DEFAULT_TARGET_WIN_COINS);
  }
  if (0 >= engineTargetFps) {
    handleParamError(ENGINE_TARGET_FPS_PARAM_NAME, engineTargetFps,
        DEFAULT_ENGINE_TARGET_FPS);
  }
  if (0 >= engineTargetFps) {
    handleParamError(ENGINE_TARGET_FPS_PARAM_NAME, engineTargetFps,
        DEFAULT_ENGINE_TARGET_FPS);
  }
  constexpr auto maxRendererFlagsMaskValue =
      getEnumValue(RendererFlag::SOFTARE_RENDERER) |
      getEnumValue(RendererFlag::HARDWARE_RENDERER) |
      getEnumValue(RendererFlag::VSYNC_ENABLE) |
      getEnumValue(RendererFlag::FBO_ENABLE);
  if (maxRendererFlagsMaskValue < rendererFlagsMask) {
    handleParamError(RENDERER_FLAGS_MASK_PARAM_NAME, rendererFlagsMask,
        DEFAULT_RENDERER_FLAGS_MASK);
  }
  const size_t maxHardwareThreads = std::thread::hardware_concurrency();
  if (ros2CommunicatorConfig.numberOfThreads > maxHardwareThreads) {
    handleParamError(ROS2_EXECUTOR_THREADS_NUM_PARAM_NAME,
        ros2CommunicatorConfig.numberOfThreads, maxHardwareThreads);
  }
}

RoboCollectorGuiRos2ParamProvider::RoboCollectorGuiRos2ParamProvider()
    : rclcpp::Node(NODE_NAME) {
  declare_parameter<int32_t>(GUI_WINDOW_X_PARAM_NAME, DEFAULT_WINDOW_X);
  declare_parameter<int32_t>(GUI_WINDOW_Y_PARAM_NAME, DEFAULT_WINDOW_Y);
  declare_parameter<int32_t>(GUI_WINDOW_WIDTH_PARAM_NAME, DEFAULT_WINDOW_WIDTH);
  declare_parameter<int32_t>(GUI_WINDOW_HEIGHT_PARAM_NAME,
      DEFAULT_WINDOW_HEIGHT);

  declare_parameter<int32_t>(ENGINE_TARGET_FPS_PARAM_NAME,
      DEFAULT_ENGINE_TARGET_FPS);
  declare_parameter<int32_t>(RENDERER_FLAGS_MASK_PARAM_NAME,
      DEFAULT_RENDERER_FLAGS_MASK);
  declare_parameter<bool>(FBO_OPTIMIZATIONS_ENABLED_PARAM_NAME,
      DEFAULT_FBO_OPTIMIZATIONS_ENABLED);

  declare_parameter<int32_t>(ROS2_EXECUTOR_TYPE_PARAM_NAME,
      DEFAULT_EXECUTOR_TYPE);
  declare_parameter<int32_t>(ROS2_EXECUTOR_THREADS_NUM_PARAM_NAME,
      DEFAULT_EXECUTOR_THREADS_NUM);

  declare_parameter<int32_t>(TOTAL_GAME_SECONDS_PARAM_NAME,
      DEFAULT_TOTAL_GAME_SECONDS);
  declare_parameter<int32_t>(TARGET_WIN_COINS_PARAM_NAME,
      DEFAULT_TARGET_WIN_COINS);
  declare_parameter<bool>(USE_LOCAL_CONTROLLER_MODE_PARAM_NAME,
      DEFAULT_USE_LOCAL_CONTROLLER_MODE);
}

RoboCollectorGuiRos2Params RoboCollectorGuiRos2ParamProvider::getParams() {
  get_parameter(GUI_WINDOW_X_PARAM_NAME, _params.guiWindow.x);
  get_parameter(GUI_WINDOW_Y_PARAM_NAME, _params.guiWindow.y);
  get_parameter(GUI_WINDOW_WIDTH_PARAM_NAME, _params.guiWindow.w);
  get_parameter(GUI_WINDOW_HEIGHT_PARAM_NAME, _params.guiWindow.h);

  get_parameter(ENGINE_TARGET_FPS_PARAM_NAME, _params.engineTargetFps);
  get_parameter(RENDERER_FLAGS_MASK_PARAM_NAME, _params.rendererFlagsMask);
  bool fboOptimizations{};
  get_parameter(FBO_OPTIMIZATIONS_ENABLED_PARAM_NAME, fboOptimizations);
  _params.fboOptimization = fboOptimizations ?
      FboOptimization::ENABLED : FboOptimization::DISABLED;

  int32_t executorTypeInt{};
  get_parameter(ROS2_EXECUTOR_TYPE_PARAM_NAME, executorTypeInt);
  _params.ros2CommunicatorConfig.executorType =
      toEnum<ExecutorType>(executorTypeInt);
  get_parameter(ROS2_EXECUTOR_THREADS_NUM_PARAM_NAME,
                _params.ros2CommunicatorConfig.numberOfThreads);

  get_parameter(TOTAL_GAME_SECONDS_PARAM_NAME, _params.totalGameSeconds);
  get_parameter(TARGET_WIN_COINS_PARAM_NAME, _params.targetWinCoins);

  bool useLocalControllerMode{};
  get_parameter(USE_LOCAL_CONTROLLER_MODE_PARAM_NAME, useLocalControllerMode);
  _params.localControrllerMode = useLocalControllerMode ?
      LocalControllerMode::ENABLED : LocalControllerMode::DISABLED;

  _params.validate();

  return _params;
}
