#ifndef ROBO_MINER_GUI_ROBOMINERGUI_H_
#define ROBO_MINER_GUI_ROBOMINERGUI_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "ros2_game_engine/communicator/Ros2CommunicatorInterface.h"
#include "game_engine/Game.h"
#include "robo_common/helpers/CollisionWatcher.h"

//Own components headers
#include "robo_miner_gui/layout/RoboMinerLayout.h"
#include "robo_miner_gui/external_api/MinerControllerExternalBridge.h"
#include "robo_miner_gui/helpers/MovementWatcher.h"
#include "robo_miner_gui/helpers/SolutionValidator.h"

//Forward declarations
class InputEvent;

class RoboMinerGui final : public Game {
public:
  friend class RoboMinerGuiInitHelper;

  RoboMinerGui(const Ros2CommunicatorInterface &communicatorOutInterface);

  ErrorCode init(const std::any &cfg) override;
  void deinit() override;

  void draw() const override;
  void handleEvent(const InputEvent &e) override;

  void process() override;

private:
  RoboMinerLayout _layout;
  CollisionWatcher _collisionWatcher;
  MovementWatcher _movementWatcher;
  SolutionValidator _solutionValidator;

  std::shared_ptr<MinerControllerExternalBridge> _controllerExternalBridge;
  Ros2CommunicatorInterface _communicatorInterface;
};

#endif /* ROBO_MINER_GUI_ROBOMINERGUI_H_ */
