//Corresponding header
#include "ur_control_gui/external_api/UrControlGuiRos2ParamProvider.h"

//System headers
#include <sstream>

//Other libraries headers
#include "utils/Log.h"

//Own components headers

namespace {
constexpr auto NODE_NAME = "UrControlGuiRos2ParamProvider";

constexpr auto GUI_WINDOW_X_PARAM_NAME = "gui_window_x";
constexpr auto GUI_WINDOW_Y_PARAM_NAME = "gui_window_y";
constexpr auto GUI_WINDOW_WIDTH_PARAM_NAME = "gui_window_width";
constexpr auto GUI_WINDOW_HEIGHT_PARAM_NAME = "gui_window_height";
constexpr auto ROBOT_IP_PARAM_NAME = "robot_ip";
constexpr auto ROBOT_INTERFACE_PORT_PARAM_NAME = "robot_interface_port";

//screen
constexpr auto DEFAULT_WINDOW_X = 72;
constexpr auto DEFAULT_WINDOW_Y = 27;
constexpr auto DEFAULT_WINDOW_WIDTH = 1848;
constexpr auto DEFAULT_WINDOW_HEIGHT = 1053;

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

void UrControlGuiRos2Params::print() const {
  std::ostringstream ostr;
  ostr << "==================================================================\n"
       << "Printing node(" << NODE_NAME << ") params:\n"
       << GUI_WINDOW_X_PARAM_NAME << ": " << guiWindow.x << '\n'
       << GUI_WINDOW_Y_PARAM_NAME << ": " << guiWindow.y << '\n'
       << GUI_WINDOW_WIDTH_PARAM_NAME << ": " << guiWindow.w << '\n'
       << GUI_WINDOW_HEIGHT_PARAM_NAME << ": " << guiWindow.h << '\n'
       << ROBOT_IP_PARAM_NAME << ": " << robotIp << '\n'
       << ROBOT_INTERFACE_PORT_PARAM_NAME << ": " << robotInterfacePort << '\n'
       << "=================================================================\n";

  LOG("%s", ostr.str().c_str());
}

void UrControlGuiRos2Params::validate() {
  if (0 >= guiWindow.w) {
    handleParamError(GUI_WINDOW_WIDTH_PARAM_NAME, guiWindow.w,
        DEFAULT_WINDOW_WIDTH);
  }
  if (0 >= guiWindow.h) {
    handleParamError(GUI_WINDOW_HEIGHT_PARAM_NAME, guiWindow.h,
        DEFAULT_WINDOW_HEIGHT);
  }

  //TODO validate ip and port
}

UrControlGuiRos2ParamProvider::UrControlGuiRos2ParamProvider()
    : rclcpp::Node(NODE_NAME) {
  declare_parameter<int32_t>(GUI_WINDOW_X_PARAM_NAME, DEFAULT_WINDOW_X);
  declare_parameter<int32_t>(GUI_WINDOW_Y_PARAM_NAME, DEFAULT_WINDOW_Y);
  declare_parameter<int32_t>(GUI_WINDOW_WIDTH_PARAM_NAME, DEFAULT_WINDOW_WIDTH);
  declare_parameter<int32_t>(GUI_WINDOW_HEIGHT_PARAM_NAME,
      DEFAULT_WINDOW_HEIGHT);
  declare_parameter<std::string>(ROBOT_IP_PARAM_NAME, DEFAULT_ROBOT_IP);
  declare_parameter<uint16_t>(ROBOT_INTERFACE_PORT_PARAM_NAME,
      DEFAULT_ROBOT_INTERFACE_PORT);
}

UrControlGuiRos2Params UrControlGuiRos2ParamProvider::getParams() {
  get_parameter(GUI_WINDOW_X_PARAM_NAME, _params.guiWindow.x);
  get_parameter(GUI_WINDOW_Y_PARAM_NAME, _params.guiWindow.y);
  get_parameter(GUI_WINDOW_WIDTH_PARAM_NAME, _params.guiWindow.w);
  get_parameter(GUI_WINDOW_HEIGHT_PARAM_NAME, _params.guiWindow.h);

  get_parameter(ROBOT_IP_PARAM_NAME, _params.robotIp);
  get_parameter(ROBOT_INTERFACE_PORT_PARAM_NAME, _params.robotInterfacePort);

  _params.validate();

  return _params;
}
