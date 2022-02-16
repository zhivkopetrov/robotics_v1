#ifndef ROBO_CLEANER_GUI_ROBOCLEANERGUI_H_
#define ROBO_CLEANER_GUI_ROBOCLEANERGUI_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "ros2_game_engine/communicator/Ros2CommunicatorInterface.h"
#include "game_engine/Game.h"
#include "robo_common/helpers/CollisionWatcher.h"

//Own components headers
#include "robo_cleaner_gui/layout/RoboCleanerLayout.h"
#include "robo_cleaner_gui/external_api/CleanerControllerExternalBridge.h"

//Forward declarations
class InputEvent;

class RoboCleanerGui final : public Game {
public:
  friend class RoboCleanerGuiInitHelper;

  RoboCleanerGui(const Ros2CommunicatorInterface &communicatorOutInterface);

  int32_t init(const std::any &cfg) override;
  void deinit() override;

  void draw() const override;
  void handleEvent(const InputEvent &e) override;

  void process() override;

  //TODO move to some object
  void onRobotTurnFinish(int32_t robotId);

private:
  RoboCleanerLayout _layout;
  CollisionWatcher _collisionWatcher;

  std::shared_ptr<CleanerControllerExternalBridge> _controllerExternalBridge;
  Ros2CommunicatorInterface _communicatorInterface;
};

#endif /* ROBO_CLEANER_GUI_ROBOCLEANERGUI_H_ */
