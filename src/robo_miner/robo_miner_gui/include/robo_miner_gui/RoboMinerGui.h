#ifndef ROBO_MINER_GUI_ROBOMINERGUI_H_
#define ROBO_MINER_GUI_ROBOMINERGUI_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "ros2_game_engine/communicator/Ros2CommunicatorInterface.h"
#include "game_engine/Game.h"
#include "robo_common/helpers/CollisionWatcher.h"

//Own components headers
#include "robo_miner_gui/layout/RoboMinerLayout.h"
#include "robo_miner_gui/external_api/MinerControllerExternalBridge.h"

//Forward declarations
class InputEvent;

class RoboMinerGui final : public Game {
public:
  friend class RoboMinerGuiInitHelper;

  RoboMinerGui(const Ros2CommunicatorInterface &communicatorOutInterface);

  int32_t init(const std::any &cfg) override;
  void deinit() override;

  void draw() const override;
  void handleEvent(const InputEvent &e) override;

  void process() override;

  //TODO move to some object
  void onRobotTurnFinish(int32_t robotId);

private:
  RoboMinerLayout _layout;
  CollisionWatcher _collisionWatcher;

  std::shared_ptr<MinerControllerExternalBridge> _controllerExternalBridge;
  Ros2CommunicatorInterface _communicatorInterface;
};

#endif /* ROBO_MINER_GUI_ROBOMINERGUI_H_ */
