#ifndef ROBO_CLEANER_GUI_ROBOCLEANERGUIROS2PARAMPROVIDER_H_
#define ROBO_CLEANER_GUI_ROBOCLEANERGUIROS2PARAMPROVIDER_H_

//System headers

//Other libraries headers
#include <rclcpp/node.hpp>
#include "robo_common/layout/field/config/FogOfWarConfig.h"
#include "utils/drawing/Rectangle.h"

//Own components headers

//Forward declarations

struct RoboCleanerGuiRos2Params {
  Rectangle guiWindow;
  int32_t levelId { };
  FogOfWarStatus fogOfWarStatus = FogOfWarStatus::ENABLED;

  void print() const;
};

class RoboCleanerGuiRos2ParamProvider : public rclcpp::Node {
public:
  RoboCleanerGuiRos2ParamProvider();

  RoboCleanerGuiRos2Params getParams();

private:
  RoboCleanerGuiRos2Params _params;
};

#endif /* ROBO_CLEANER_GUI_ROBOCLEANERGUIROS2PARAMPROVIDER_H_ */
