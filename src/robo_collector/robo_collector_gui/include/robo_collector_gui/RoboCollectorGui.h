#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUI_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUI_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <array>

//Other libraries headers
#include "ros2_game_engine/communicator/Ros2CommunicatorInterface.h"
#include "game_engine/Game.h"
#include "robo_common/helpers/CollisionWatcher.h"

//Own components headers
#include "robo_collector_gui/layout/RoboCollectorLayout.h"
#include "robo_collector_gui/helpers/TurnHelper.h"
#include "robo_collector_gui/external_api/ControllerExternalBridge.h"

//Forward declarations
class InputEvent;
struct RoboCollectorLayoutConfig;
struct RoboCollectorGuiConfig;
struct RoboMinerGuiConfig;

class RoboCollectorGui final : public Game {
public:
  RoboCollectorGui(const Ros2CommunicatorInterface &communicatorOutInterface);

  int32_t init(const std::any &cfg) override;
  void deinit() override;

  void draw() const override;
  void handleEvent(const InputEvent &e) override;

  void process() override;

private:
  int32_t initLayout(const RoboCollectorLayoutConfig &cfg);
  int32_t initTurnHelper(const RoboCollectorLayoutInterface &interface,
                         char fieldEnemyMarker);
  int32_t initControllerExternalBridge(
      const RoboCollectorLayoutInterface &interface);

  RoboCollectorLayout _layout;
  TurnHelper _turnHelper;
  CollisionWatcher _collisionWatcher;

  std::shared_ptr<ControllerExternalBridge> _controllerExternalBridge;

  Ros2CommunicatorInterface _communicatorOutInterface;
};

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUI_H_ */
