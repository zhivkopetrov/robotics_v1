#ifndef ROBO_CLEANER_GUI_ROBOCLEANERGUI_H_
#define ROBO_CLEANER_GUI_ROBOCLEANERGUI_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "ros2_game_engine/communicator/Ros2CommunicatorInterface.h"
#include "game_engine/Game.h"
#include "robo_common/helpers/CollisionWatcher.h"

//Own components headers
#include "robo_cleaner_gui/layout/RoboCleanerLayout.h"
#include "robo_cleaner_gui/external_api/CleanerControllerExternalBridge.h"
#include "robo_cleaner_gui/helpers/MovementWatcher.h"
#include "robo_cleaner_gui/external_api/MovementReporter.h"
#include "robo_cleaner_gui/helpers/RoboCleanerSolutionValidator.h"

//Forward declarations
class InputEvent;

class RoboCleanerGui final : public Game {
public:
  friend class RoboCleanerGuiInitHelper;

  RoboCleanerGui(const Ros2CommunicatorInterface &communicatorOutInterface);

  ErrorCode init(const std::any &cfg) override;
  void deinit() override;

  void draw() const override;
  void handleEvent(const InputEvent &e) override;

  void process() override;

private:
  RoboCleanerLayout _layout;
  CollisionWatcher _collisionWatcher;
  MovementWatcher _movementWatcher;
  MovementReporter _movementReporter;
  RoboCleanerSolutionValidator _solutionValidator;

  std::shared_ptr<CleanerControllerExternalBridge> _controllerExternalBridge;
  Ros2CommunicatorInterface _communicatorInterface;
};

#endif /* ROBO_CLEANER_GUI_ROBOCLEANERGUI_H_ */
