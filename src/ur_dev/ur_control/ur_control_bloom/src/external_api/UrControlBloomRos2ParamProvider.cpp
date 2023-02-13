//Corresponding header
#include "ur_control_bloom/external_api/UrControlBloomRos2ParamProvider.h"

//System headers
#include <sstream>
#include <thread>

//Other libraries headers
#include "utils/data_type/EnumClassUtils.h"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/Log.h"

//Own components headers

namespace {
constexpr auto NODE_NAME = "UrControlBloomRos2ParamProvider";

constexpr auto GUI_WINDOW_X_PARAM_NAME = "gui_window_x";
constexpr auto GUI_WINDOW_Y_PARAM_NAME = "gui_window_y";
constexpr auto GUI_WINDOW_WIDTH_PARAM_NAME = "gui_window_width";
constexpr auto GUI_WINDOW_HEIGHT_PARAM_NAME = "gui_window_height";
constexpr auto GRIPPER_TYPE_PARAM_NAME = "gripper_type";
constexpr auto ROS2_EXECUTOR_TYPE_PARAM_NAME = "ros2_executor_type";
constexpr auto ROS2_EXECUTOR_THREADS_NUM_PARAM_NAME = "ros2_executor_threads_num";
constexpr auto ROBOT_IP_PARAM_NAME = "robot_ip";
constexpr auto ROBOT_INTERFACE_PORT_PARAM_NAME = "robot_interface_port";

//screen
constexpr auto DEFAULT_WINDOW_X = 72;
constexpr auto DEFAULT_WINDOW_Y = 27;
constexpr auto DEFAULT_WINDOW_WIDTH = 1848;
constexpr auto DEFAULT_WINDOW_HEIGHT = 1053;

//ROS2 executor
constexpr auto DEFAULT_EXECUTOR_TYPE = 0;
constexpr auto DEFAULT_EXECUTOR_THREADS_NUM = 2;

//gripper
constexpr auto DEFAULT_GRIPPER_TYPE = getEnumValue(GripperType::SIMULATION);

//robot
constexpr auto DEFAULT_ROBOT_IP = "192.168.1.102";
constexpr uint16_t DEFAULT_ROBOT_INTERFACE_PORT = 30002;

template <typename T>
void handleParamError(const char *paramName, T &value, const T &defaultValue) {
  std::ostringstream ostr;
  ostr << "Param: [" << paramName << "] has invalid value: [" << value
  << "]. Overriding with default value: [" << defaultValue << "]";
  LOGR("%s", ostr.str().c_str());

  value = defaultValue;
}
}

void UrControlBloomRos2Params::print() const {
  std::ostringstream ostr;
  ostr << "==================================================================\n"
       << "Printing node(" << NODE_NAME << ") params:\n"
       << GUI_WINDOW_X_PARAM_NAME << ": " << guiWindow.x << '\n'
       << GUI_WINDOW_Y_PARAM_NAME << ": " << guiWindow.y << '\n'
       << GUI_WINDOW_WIDTH_PARAM_NAME << ": " << guiWindow.w << '\n'
       << GUI_WINDOW_HEIGHT_PARAM_NAME << ": " << guiWindow.h << '\n'
       << GUI_WINDOW_HEIGHT_PARAM_NAME << ": " << guiWindow.h << '\n'
       << GRIPPER_TYPE_PARAM_NAME << ": " <<
         (GripperType::HARDWARE == gripperType ? "Hardware" : "Simulation") 
          << '\n'
       << ROS2_EXECUTOR_TYPE_PARAM_NAME << ": " <<
         getExecutorName(ros2CommunicatorConfig.executorType) << '\n'
       << ROS2_EXECUTOR_THREADS_NUM_PARAM_NAME << ": "
         << ros2CommunicatorConfig.numberOfThreads << '\n'
       << ROBOT_IP_PARAM_NAME << ": " << robotIp << '\n'
       << ROBOT_INTERFACE_PORT_PARAM_NAME << ": " << robotInterfacePort << '\n'
       << "=================================================================\n";

  LOG("%s", ostr.str().c_str());
}

void UrControlBloomRos2Params::validate() {
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
  if ((GripperType::HARDWARE != gripperType) && 
      (GripperType::SIMULATION != gripperType)) {
    LOGERR("Error, received unsupported GripperType: [%d]. Defaulting to "
           "GripperType::SIMULATION", getEnumValue(gripperType));
    gripperType = GripperType::SIMULATION;
  }

  //TODO validate ip and port
}

UrControlBloomRos2ParamProvider::UrControlBloomRos2ParamProvider()
    : rclcpp::Node(NODE_NAME) {
  declare_parameter<int32_t>(GUI_WINDOW_X_PARAM_NAME, DEFAULT_WINDOW_X);
  declare_parameter<int32_t>(GUI_WINDOW_Y_PARAM_NAME, DEFAULT_WINDOW_Y);
  declare_parameter<int32_t>(GUI_WINDOW_WIDTH_PARAM_NAME, DEFAULT_WINDOW_WIDTH);
  declare_parameter<int32_t>(GUI_WINDOW_HEIGHT_PARAM_NAME,
      DEFAULT_WINDOW_HEIGHT);

  declare_parameter<int32_t>(GRIPPER_TYPE_PARAM_NAME, DEFAULT_GRIPPER_TYPE);

  declare_parameter<int32_t>(ROS2_EXECUTOR_TYPE_PARAM_NAME,
      DEFAULT_EXECUTOR_TYPE);
  declare_parameter<int32_t>(ROS2_EXECUTOR_THREADS_NUM_PARAM_NAME,
      DEFAULT_EXECUTOR_THREADS_NUM);

  declare_parameter<std::string>(ROBOT_IP_PARAM_NAME, DEFAULT_ROBOT_IP);
  declare_parameter<uint16_t>(ROBOT_INTERFACE_PORT_PARAM_NAME,
      DEFAULT_ROBOT_INTERFACE_PORT);
}

UrControlBloomRos2Params UrControlBloomRos2ParamProvider::getParams() {
  get_parameter(GUI_WINDOW_X_PARAM_NAME, _params.guiWindow.x);
  get_parameter(GUI_WINDOW_Y_PARAM_NAME, _params.guiWindow.y);
  get_parameter(GUI_WINDOW_WIDTH_PARAM_NAME, _params.guiWindow.w);
  get_parameter(GUI_WINDOW_HEIGHT_PARAM_NAME, _params.guiWindow.h);

  int32_t gripperTypeInt{};
  get_parameter(GRIPPER_TYPE_PARAM_NAME, gripperTypeInt);
  _params.gripperType = toEnum<GripperType>(gripperTypeInt);

  int32_t executorTypeInt{};
  get_parameter(ROS2_EXECUTOR_TYPE_PARAM_NAME, executorTypeInt);
  _params.ros2CommunicatorConfig.executorType =
      toEnum<ExecutorType>(executorTypeInt);
  get_parameter(ROS2_EXECUTOR_THREADS_NUM_PARAM_NAME,
                _params.ros2CommunicatorConfig.numberOfThreads);

  get_parameter(ROBOT_IP_PARAM_NAME, _params.robotIp);
  get_parameter(ROBOT_INTERFACE_PORT_PARAM_NAME, _params.robotInterfacePort);

  _params.validate();

  return _params;
}
