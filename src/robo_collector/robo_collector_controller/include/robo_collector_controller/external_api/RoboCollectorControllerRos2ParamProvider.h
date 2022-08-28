#ifndef ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERROS2PARAMPROVIDER_H_
#define ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERROS2PARAMPROVIDER_H_

//System headers

//Other libraries headers
#include <rclcpp/node.hpp>
#include "robo_common/defines/RoboCommonDefines.h"
#include "robo_collector_common/layout/controller/config/RoboCollectorUiControllerBaseConfig.h"
#include "ros2_game_engine/communicator/config/Ros2CommunicatorConfig.h"
#include "utils/drawing/Rectangle.h"

//Own components headers

//Forward declarations

struct RoboCollectorControllerRos2Params {
  Rectangle guiWindow;
  Ros2CommunicatorConfig ros2CommunicatorConfig;
  LocalControllerMode localControrllerMode = LocalControllerMode::DISABLED;
  UserData userData;

  void print() const;
  void validate();
};

class RoboCollectorControllerRos2ParamProvider : public rclcpp::Node {
public:
  RoboCollectorControllerRos2ParamProvider();

  RoboCollectorControllerRos2Params getParams();

private:
  RoboCollectorControllerRos2Params _params;
};

#endif /* ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERROS2PARAMPROVIDER_H_ */
