#ifndef UR_CONTROL_GUI_URCONTROLGUIBUILDER_H_
#define UR_CONTROL_GUI_URCONTROLGUIBUILDER_H_

//System headers
#include <cstdint>
#include <memory>

//Other libraries headers

//Own components headers
#include "ur_control_gui/UrControlGui.h"

//Forward declarations
class Ros2Communicator;

class UrControlGuiBuilder {
public:
  UrControlGuiBuilder() = delete;

  static std::unique_ptr<UrControlGui> createUrControlGui(
      const std::unique_ptr<Ros2Communicator>& communicator);
};

#endif /* UR_CONTROL_GUI_URCONTROLGUIBUILDER_H_ */
