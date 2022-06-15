#ifndef ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERROS2PARAMPROVIDER_H_
#define ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERROS2PARAMPROVIDER_H_

//System headers

//Other libraries headers
#include <rclcpp/node.hpp>
#include "robo_collector_common/layout/controller/config/RoboCollectorUiControllerBaseConfig.h"
#include "utils/drawing/Rectangle.h"

//Own components headers

//Forward declarations

struct RoboCollectorControllerRos2Params {
  Rectangle guiWindow;
  LocalControllerMode localControrllerMode = LocalControllerMode::DISABLED;

  void print() const;
};

class RoboCollectorControllerRos2ParamProvider : public rclcpp::Node {
public:
  RoboCollectorControllerRos2ParamProvider();

  RoboCollectorControllerRos2Params getParams();

private:
  RoboCollectorControllerRos2Params _params;
};

#endif /* ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERROS2PARAMPROVIDER_H_ */
