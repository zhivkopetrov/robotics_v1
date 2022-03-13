#ifndef ROBO_MINER_GUI_ROBOMINERBUILDER_H_
#define ROBO_MINER_GUI_ROBOMINERBUILDER_H_

//System headers
#include <cstdint>
#include <memory>

//Other libraries headers

//Own components headers
#include "robo_miner_gui/RoboMinerGui.h"

//Forward declarations
class Ros2Communicator;

class RoboMinerBuilder {
public:
  RoboMinerBuilder() = delete;

  static std::unique_ptr<RoboMinerGui> createRoboMinerGui(
      const std::unique_ptr<Ros2Communicator>& communicator);
};

#endif /* ROBO_MINER_GUI_ROBOMINERBUILDER_H_ */
