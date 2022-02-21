#ifndef ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERBUILDER_H_
#define ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERBUILDER_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <memory>

//Other libraries headers

//Own components headers
#include "robo_collector_controller/RoboCollectorController.h"

//Forward declarations
class Ros2Communicator;

class RoboCollectorControllerBuilder {
public:
  RoboCollectorControllerBuilder() = delete;

  static std::unique_ptr<RoboCollectorController> createRoboCollectorController(
      const std::unique_ptr<Ros2Communicator>& communicator);
};

#endif /* ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERBUILDER_H_ */
