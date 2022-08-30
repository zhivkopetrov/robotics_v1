//Corresponding header
#include "robo_miner_gui/external_api/RoboMinerGuiRos2ParamProvider.h"

//System headers
#include <sstream>
#include <thread>

//Other libraries headers
#include "utils/data_type/EnumClassUtils.h"
#include "utils/Log.h"

//Own components headers

namespace {
constexpr auto NODE_NAME = "RoboMinerGuiRos2ParamProvider";

constexpr auto GUI_WINDOW_X_PARAM_NAME = "gui_window_x";
constexpr auto GUI_WINDOW_Y_PARAM_NAME = "gui_window_y";
constexpr auto GUI_WINDOW_WIDTH_PARAM_NAME = "gui_window_width";
constexpr auto GUI_WINDOW_HEIGHT_PARAM_NAME = "gui_window_height";
constexpr auto ROS2_EXECUTOR_TYPE_PARAM_NAME = "ros2_executor_type";
constexpr auto ROS2_EXECUTOR_THREADS_NUM_PARAM_NAME = "ros2_executor_threads_num";
constexpr auto LEVEL_ID_PARAM_NAME = "level_id";
constexpr auto USE_FOG_OF_WAR_PARAM_NAME = "use_fog_of_war";

//screen
constexpr auto DEFAULT_WINDOW_X = 72;
constexpr auto DEFAULT_WINDOW_Y = 27;
constexpr auto DEFAULT_WINDOW_WIDTH = 1848;
constexpr auto DEFAULT_WINDOW_HEIGHT = 1053;

//ROS2 executor
constexpr auto DEFAULT_EXECUTOR_TYPE = 0;
constexpr auto DEFAULT_EXECUTOR_THREADS_NUM = 2;

//misc
constexpr auto DEFAULT_LEVEL_ID = 1;
constexpr auto DEFAULT_USE_FOG_OF_WAR = true;

constexpr auto MAX_SUPPORTED_LEVEL_ID = 3;

template<typename T>
void handleParamError(const char* paramName, T& value, const T& defaultValue) {
  std::ostringstream ostr;
  ostr << "Param: [" << paramName << "] has invalid value: [" << value
       << "]. Overriding with default value: [" << defaultValue << "]";
  LOGR("%s", ostr.str().c_str());

  value = defaultValue;
}
}

void RoboMinerGuiRos2Params::print() const {
  std::ostringstream ostr;
  ostr << "==================================================================\n"
       << "Printing node(" << NODE_NAME << ") params:\n"
       << GUI_WINDOW_X_PARAM_NAME << ": " << guiWindow.x << '\n'
       << GUI_WINDOW_Y_PARAM_NAME << ": " << guiWindow.y << '\n'
       << GUI_WINDOW_WIDTH_PARAM_NAME << ": " << guiWindow.w << '\n'
       << GUI_WINDOW_HEIGHT_PARAM_NAME << ": " << guiWindow.h << '\n'
       << ROS2_EXECUTOR_TYPE_PARAM_NAME << ": " <<
         getExecutorName(ros2CommunicatorConfig.executorType) << '\n'
       << ROS2_EXECUTOR_THREADS_NUM_PARAM_NAME << ": "
         << ros2CommunicatorConfig.numberOfThreads << '\n'
       << LEVEL_ID_PARAM_NAME << ": " << levelId << '\n'
       << USE_FOG_OF_WAR_PARAM_NAME << ": "
           << ((FogOfWarStatus::ENABLED == fogOfWarStatus) ?
               "true" : "false") << '\n'
       << "=================================================================\n";

  LOG("%s", ostr.str().c_str());
}

void RoboMinerGuiRos2Params::validate() {
  if (0 >= guiWindow.w) {
    handleParamError(GUI_WINDOW_WIDTH_PARAM_NAME, guiWindow.w,
        DEFAULT_WINDOW_WIDTH);
  }
  if (0 >= guiWindow.h) {
    handleParamError(GUI_WINDOW_HEIGHT_PARAM_NAME, guiWindow.h,
        DEFAULT_WINDOW_HEIGHT);
  }
  if ((0 >= levelId) || (MAX_SUPPORTED_LEVEL_ID < levelId)) {
    handleParamError(LEVEL_ID_PARAM_NAME, levelId, DEFAULT_LEVEL_ID);
  }
  const size_t maxHardwareThreads = std::thread::hardware_concurrency();
  if (ros2CommunicatorConfig.numberOfThreads > maxHardwareThreads) {
    handleParamError(ROS2_EXECUTOR_THREADS_NUM_PARAM_NAME,
        ros2CommunicatorConfig.numberOfThreads, maxHardwareThreads);
  }
}

RoboMinerGuiRos2ParamProvider::RoboMinerGuiRos2ParamProvider()
    : rclcpp::Node(NODE_NAME) {
  declare_parameter<int32_t>(GUI_WINDOW_X_PARAM_NAME, DEFAULT_WINDOW_X);
  declare_parameter<int32_t>(GUI_WINDOW_Y_PARAM_NAME, DEFAULT_WINDOW_Y);
  declare_parameter<int32_t>(GUI_WINDOW_WIDTH_PARAM_NAME, DEFAULT_WINDOW_WIDTH);
  declare_parameter<int32_t>(GUI_WINDOW_HEIGHT_PARAM_NAME,
      DEFAULT_WINDOW_HEIGHT);

  declare_parameter<int32_t>(ROS2_EXECUTOR_TYPE_PARAM_NAME,
      DEFAULT_EXECUTOR_TYPE);
  declare_parameter<int32_t>(ROS2_EXECUTOR_THREADS_NUM_PARAM_NAME,
      DEFAULT_EXECUTOR_THREADS_NUM);

  declare_parameter<int32_t>(LEVEL_ID_PARAM_NAME, DEFAULT_LEVEL_ID);
  declare_parameter<bool>(USE_FOG_OF_WAR_PARAM_NAME, DEFAULT_USE_FOG_OF_WAR);
}

RoboMinerGuiRos2Params RoboMinerGuiRos2ParamProvider::getParams() {
  get_parameter(GUI_WINDOW_X_PARAM_NAME, _params.guiWindow.x);
  get_parameter(GUI_WINDOW_Y_PARAM_NAME, _params.guiWindow.y);
  get_parameter(GUI_WINDOW_WIDTH_PARAM_NAME, _params.guiWindow.w);
  get_parameter(GUI_WINDOW_HEIGHT_PARAM_NAME, _params.guiWindow.h);

  int32_t executorTypeInt{};
  get_parameter(ROS2_EXECUTOR_TYPE_PARAM_NAME, executorTypeInt);
  _params.ros2CommunicatorConfig.executorType =
      toEnum<ExecutorType>(executorTypeInt);
  get_parameter(ROS2_EXECUTOR_THREADS_NUM_PARAM_NAME,
                _params.ros2CommunicatorConfig.numberOfThreads);

  get_parameter(LEVEL_ID_PARAM_NAME, _params.levelId);

  bool useFogOfWar{};
  get_parameter(USE_FOG_OF_WAR_PARAM_NAME, useFogOfWar);
  _params.fogOfWarStatus = useFogOfWar ?
      FogOfWarStatus::ENABLED : FogOfWarStatus::DISABLED;

  _params.validate();

  return _params;
}
