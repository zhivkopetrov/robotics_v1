//Corresponding header
#include "urscript_bridge/external_api/UrBridgeRos2ParamProvider.h"

//System headers
#include <sstream>
#include <iomanip>
#include <thread>

//Other libraries headers
#include "utils/data_type/EnumClassUtils.h"
#include "utils/Log.h"

//Own components headers

namespace {
constexpr auto NODE_NAME = "UrBridgeRos2ParamProvider";

constexpr auto VERBOSE_LOGGING_PARAM_NAME = "verbose_logging";

constexpr auto ROS2_EXECUTOR_TYPE_PARAM_NAME = "ros2_executor_type";
constexpr auto ROS2_EXECUTOR_THREADS_NUM_PARAM_NAME =
    "ros2_executor_threads_num";

constexpr auto ROBOT_IP_PARAM_NAME = "robot_ip";
constexpr auto ROBOT_INTERFACE_PORT_PARAM_NAME = "robot_interface_port";
constexpr auto URSCRIPT_SERVICE_READY_PIN_PARAM_NAME =
    "urscript_service_ready_pin";

//misc
constexpr bool DEFAULT_VERBOSE_LOGGING = false;

//ROS2 executor
constexpr auto DEFAULT_EXECUTOR_TYPE = 0;
constexpr auto DEFAULT_EXECUTOR_THREADS_NUM = 2;

//robot
constexpr auto DEFAULT_ROBOT_IP = "192.168.1.102";
constexpr uint16_t DEFAULT_ROBOT_INTERFACE_PORT = 30003;
constexpr uint32_t DEFAULT_URSCRIPT_SERVICE_READY_PIN = 4;

template <typename T>
void handleParamError(const char *paramName, T &value, const T &defaultValue) {
  std::ostringstream ostr;
  ostr << "Param: [" << paramName << "] has invalid value: [" << value
  << "]. Overriding with default value: [" << defaultValue << "]";
  LOGR("%s", ostr.str().c_str());

  value = defaultValue;
}
}

void UrBridgeRos2Params::print() const {
  std::ostringstream ostr;
  ostr << "==================================================================\n"
       << "Printing node(" << NODE_NAME << ") params:\n"
       << VERBOSE_LOGGING_PARAM_NAME << ": " << std::boolalpha << 
          verboseLogging << '\n'
       << ROS2_EXECUTOR_TYPE_PARAM_NAME << ": " <<
         getExecutorName(ros2CommunicatorCfg.executorType) << '\n'
       << ROS2_EXECUTOR_THREADS_NUM_PARAM_NAME << ": "
         << ros2CommunicatorCfg.numberOfThreads << '\n'
       << ROBOT_IP_PARAM_NAME << ": " << robotIp << '\n'
       << ROBOT_INTERFACE_PORT_PARAM_NAME << ": " << robotInterfacePort << '\n'
       << URSCRIPT_SERVICE_READY_PIN_PARAM_NAME << ": "
         << urScriptServiceReadyPin << '\n'
       << "=================================================================\n";

  LOG("%s", ostr.str().c_str());
}

void UrBridgeRos2Params::validate() {
  const size_t maxHardwareThreads = std::thread::hardware_concurrency();
  if (ros2CommunicatorCfg.numberOfThreads > maxHardwareThreads) {
    handleParamError(ROS2_EXECUTOR_THREADS_NUM_PARAM_NAME,
        ros2CommunicatorCfg.numberOfThreads, maxHardwareThreads);
  }

  //pin 0 is reserved for aborting (overriding URScripts)
  constexpr uint32_t reservedPinIdx = 0;

  //per documentation 
  //https://s3-eu-west-1.amazonaws.com/ur-support-site/46196/scriptManual.pdf
  constexpr uint32_t maxPinIdx = 7;
  if ((urScriptServiceReadyPin > maxPinIdx) || 
      (reservedPinIdx == urScriptServiceReadyPin)) {
    handleParamError(URSCRIPT_SERVICE_READY_PIN_PARAM_NAME,
        urScriptServiceReadyPin, DEFAULT_URSCRIPT_SERVICE_READY_PIN);
  }

  //TODO validate ip and port
}

UrBridgeRos2ParamProvider::UrBridgeRos2ParamProvider()
    : rclcpp::Node(NODE_NAME) {
  declare_parameter<bool>(VERBOSE_LOGGING_PARAM_NAME,
      DEFAULT_VERBOSE_LOGGING);

  declare_parameter<int32_t>(ROS2_EXECUTOR_TYPE_PARAM_NAME,
      DEFAULT_EXECUTOR_TYPE);
  declare_parameter<int32_t>(ROS2_EXECUTOR_THREADS_NUM_PARAM_NAME,
      DEFAULT_EXECUTOR_THREADS_NUM);

  declare_parameter<std::string>(ROBOT_IP_PARAM_NAME, DEFAULT_ROBOT_IP);
  declare_parameter<uint16_t>(ROBOT_INTERFACE_PORT_PARAM_NAME,
      DEFAULT_ROBOT_INTERFACE_PORT);
  declare_parameter<int32_t>(URSCRIPT_SERVICE_READY_PIN_PARAM_NAME,
      DEFAULT_URSCRIPT_SERVICE_READY_PIN);
}

UrBridgeRos2Params UrBridgeRos2ParamProvider::getParams() {
  get_parameter(VERBOSE_LOGGING_PARAM_NAME, _params.verboseLogging);

  int32_t executorTypeInt{};
  get_parameter(ROS2_EXECUTOR_TYPE_PARAM_NAME, executorTypeInt);
  _params.ros2CommunicatorCfg.executorType =
      toEnum<ExecutorType>(executorTypeInt);
  get_parameter(ROS2_EXECUTOR_THREADS_NUM_PARAM_NAME,
                _params.ros2CommunicatorCfg.numberOfThreads);

  get_parameter(ROBOT_IP_PARAM_NAME, _params.robotIp);
  get_parameter(ROBOT_INTERFACE_PORT_PARAM_NAME, _params.robotInterfacePort);
  get_parameter(URSCRIPT_SERVICE_READY_PIN_PARAM_NAME,
                _params.urScriptServiceReadyPin);

  _params.validate();

  return _params;
}
