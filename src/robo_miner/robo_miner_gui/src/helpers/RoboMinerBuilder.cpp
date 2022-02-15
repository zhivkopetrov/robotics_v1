//Corresponding header
#include "robo_miner_gui/helpers/RoboMinerBuilder.h"

//C system headers

//C++ system headers

//Other libraries headers

//Own components headers
#include "ros2_game_engine/communicator/Ros2Communicator.h"
#include "ros2_game_engine/communicator/Ros2CommunicatorInterface.h"

std::unique_ptr<RoboMinerGui> RoboMinerBuilder::createRoboMinerGui(
    const std::unique_ptr<Ros2Communicator>& communicator) {
  using namespace std::placeholders;
  Ros2CommunicatorInterface interface;
  interface.registerNodeCb =
      std::bind(&Ros2Communicator::registerNode, communicator.get(), _1);
  interface.unregisterNodeCb =
      std::bind(&Ros2Communicator::unregisterNode, communicator.get(), _1);

  return std::make_unique<RoboMinerGui>(interface);
}
