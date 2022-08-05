#ifndef UR_CONTROL_GUI_URCONTROLGUIROS2PARAMPROVIDER_H_
#define UR_CONTROL_GUI_URCONTROLGUIROS2PARAMPROVIDER_H_

//System headers
#include <string>

//Other libraries headers
#include <rclcpp/node.hpp>
#include "utils/drawing/Rectangle.h"

//Own components headers

//Forward declarations

struct UrControlGuiRos2Params {
  Rectangle guiWindow;
  std::string robotIp;
  uint16_t robotInterfacePort {};

  void print() const;
  void validate();
};

class UrControlGuiRos2ParamProvider : public rclcpp::Node {
public:
  UrControlGuiRos2ParamProvider();

  UrControlGuiRos2Params getParams();

private:
  UrControlGuiRos2Params _params;
};

#endif /* UR_CONTROL_GUI_URCONTROLGUIROS2PARAMPROVIDER_H_ */
