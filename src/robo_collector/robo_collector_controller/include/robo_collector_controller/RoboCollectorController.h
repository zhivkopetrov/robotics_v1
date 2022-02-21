#ifndef ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLER_H_
#define ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLER_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "ros2_game_engine/communicator/Ros2CommunicatorInterface.h"
#include "game_engine/Game.h"

//Own components headers
#include "robo_collector_controller/layout/RoboCollectorControllerLayout.h"
#include "robo_collector_controller/external_api/CollectorGuiExternalBridge.h"

//Forward declarations
class InputEvent;

class RoboCollectorController final : public Game {
public:
  friend class RoboCollectorControllerInitHelper;

  RoboCollectorController(
      const Ros2CommunicatorInterface &communicatorOutInterface);

  int32_t init(const std::any &cfg) override;
  void deinit() override;

  void draw() const override;
  void handleEvent(const InputEvent &e) override;

  void process() override;

private:
  RoboCollectorControllerLayout _layout;

  std::shared_ptr<CollectorGuiExternalBridge> _controllerExternalBridge;
  Ros2CommunicatorInterface _communicatorInterface;
};

#endif /* ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLER_H_ */
