//Corresponding header
#include "robo_collector_controller/external_api/RoboCollectorControllerRos2ParamProvider.h"

//System headers
#include <sstream>
#include <thread>

//Other libraries headers
#include "utils/data_type/EnumClassUtils.h"
#include "utils/Log.h"

//Own components headers

namespace {
constexpr auto NODE_NAME = "RoboCollectorControllerRos2ParamProvider";

constexpr auto GUI_WINDOW_X_PARAM_NAME = "gui_window_x";
constexpr auto GUI_WINDOW_Y_PARAM_NAME = "gui_window_y";
constexpr auto GUI_WINDOW_WIDTH_PARAM_NAME = "gui_window_width";
constexpr auto GUI_WINDOW_HEIGHT_PARAM_NAME = "gui_window_height";
constexpr auto ROS2_EXECUTOR_TYPE_PARAM_NAME = "ros2_executor_type";
constexpr auto ROS2_EXECUTOR_THREADS_NUM_PARAM_NAME =
    "ros2_executor_threads_num";
constexpr auto USE_LOCAL_CONTROLLER_MODE_PARAM_NAME =
    "use_local_controller_mode";
constexpr auto USER_PARAM_NAME = "user";
constexpr auto REPOSITORY_PARAM_NAME = "repository";
constexpr auto COMMIT_SHA_PARAM_NAME = "commit_sha";

//screen
constexpr auto DEFAULT_WINDOW_X = 1272;
constexpr auto DEFAULT_WINDOW_Y = 527;
constexpr auto DEFAULT_WINDOW_WIDTH = 648;
constexpr auto DEFAULT_WINDOW_HEIGHT = 553;

//ROS2 executor
constexpr auto DEFAULT_EXECUTOR_TYPE = 0;
constexpr auto DEFAULT_EXECUTOR_THREADS_NUM = 2;

//misc
constexpr auto DEFAULT_USE_LOCAL_CONTROLLER_MODE = true;

//user data
constexpr auto DEFAULT_USER = "not_set";
constexpr auto DEFAULT_REPOSITORY = "not_set";
constexpr auto DEFAULT_COMMIT_SHA = "not_set";

template<typename T>
void handleParamError(const char* paramName, T& value, const T& defaultValue) {
  std::ostringstream ostr;
  ostr << "Param: [" << paramName << "] has invalid value: [" << value
       << "]. Overriding with default value: [" << defaultValue << "]";
  LOGR("%s", ostr.str().c_str());

  value = defaultValue;
}
}

void RoboCollectorControllerRos2Params::print() const {
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
       << USE_LOCAL_CONTROLLER_MODE_PARAM_NAME << ": "
           << ((LocalControllerMode::ENABLED == localControrllerMode) ?
               "true" : "false") << '\n'
       << USER_PARAM_NAME << ": " << userData.user << '\n'
       << REPOSITORY_PARAM_NAME << ": " << userData.repository << '\n'
       << COMMIT_SHA_PARAM_NAME << ": " << userData.commitSha << '\n'
       << "=================================================================\n";

  LOG("%s", ostr.str().c_str());
}

void RoboCollectorControllerRos2Params::validate() {
  if (0 >= guiWindow.w) {
    handleParamError(GUI_WINDOW_WIDTH_PARAM_NAME, guiWindow.w,
        DEFAULT_WINDOW_WIDTH);
  }
  if (0 >= guiWindow.h) {
    handleParamError(GUI_WINDOW_HEIGHT_PARAM_NAME, guiWindow.h,
        DEFAULT_WINDOW_HEIGHT);
  }
  const size_t maxHardwareThreads = std::thread::hardware_concurrency();
  if (ros2CommunicatorConfig.numberOfThreads > maxHardwareThreads) {
    handleParamError(ROS2_EXECUTOR_THREADS_NUM_PARAM_NAME,
        ros2CommunicatorConfig.numberOfThreads, maxHardwareThreads);
  }
}

RoboCollectorControllerRos2ParamProvider::RoboCollectorControllerRos2ParamProvider()
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

  declare_parameter<bool>(USE_LOCAL_CONTROLLER_MODE_PARAM_NAME,
      DEFAULT_USE_LOCAL_CONTROLLER_MODE);
  declare_parameter<std::string>(USER_PARAM_NAME, DEFAULT_USER);
  declare_parameter<std::string>(REPOSITORY_PARAM_NAME, DEFAULT_REPOSITORY);
  declare_parameter<std::string>(COMMIT_SHA_PARAM_NAME, DEFAULT_COMMIT_SHA);
}

RoboCollectorControllerRos2Params RoboCollectorControllerRos2ParamProvider::getParams() {
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

  bool useLocalControllerMode{};
  get_parameter(USE_LOCAL_CONTROLLER_MODE_PARAM_NAME, useLocalControllerMode);
  _params.localControrllerMode = useLocalControllerMode ?
      LocalControllerMode::ENABLED : LocalControllerMode::DISABLED;

  get_parameter(USER_PARAM_NAME, _params.userData.user);
  get_parameter(REPOSITORY_PARAM_NAME, _params.userData.repository);
  get_parameter(COMMIT_SHA_PARAM_NAME, _params.userData.commitSha);

  _params.validate();

  return _params;
}
