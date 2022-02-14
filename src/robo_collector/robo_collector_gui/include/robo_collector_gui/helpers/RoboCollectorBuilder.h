#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORBUILDER_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORBUILDER_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <memory>

//Other libraries headers

//Own components headers
#include "robo_collector_gui/RoboCollectorGui.h"

//Forward declarations
class Ros2Communicator;

class RoboCollectorBuilder {
public:
  RoboCollectorBuilder() = delete;

  static std::unique_ptr<RoboCollectorGui> createRoboCollectorGui(
      const std::unique_ptr<Ros2Communicator>& communicator);
};

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORBUILDER_H_ */
