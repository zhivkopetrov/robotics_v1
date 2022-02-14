#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUI_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUI_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <array>

//Other libraries headers
#include "ros2_game_engine/communicator/Ros2CommunicatorInterface.h"
#include "game_engine/Game.h"
#include "robo_common/field/Field.h"

//Own components headers
#include "robo_collector_gui/entities/robot/Robot.h"
#include "robo_collector_gui/panels/PanelHandler.h"
#include "robo_collector_gui/entities/coin/CoinHandler.h"
#include "robo_collector_gui/helpers/CollisionWatcher.h"
#include "robo_collector_gui/helpers/TurnHelper.h"
#include "robo_collector_gui/helpers/GameEndHelper.h"
#include "robo_collector_gui/controller/RoboCollectorController.h"
#include "robo_collector_gui/external_api/ControllerExternalBridge.h"

#include "robo_collector_gui/robo_miner/RoboMinerGui.h"
#include "robo_collector_gui/robo_cleaner/RoboCleanerGui.h"

//Forward declarations
class InputEvent;
struct RoboCollectorGuiConfig;
struct CoinHandlerConfig;
struct RoboCollectorControllerConfig;

struct RoboMinerGuiConfig;

class RoboCollectorGui final : public Game {
public:
  RoboCollectorGui(const Ros2CommunicatorInterface& communicatorOutInterface);

  int32_t init(const std::any& cfg) override;
  void deinit() override;

  void draw() const override;
  void handleEvent(const InputEvent &e) override;

  void process() override;

  //TODO create a separate help page class
  void activateHelpPage();

  void changeGameType(GameType gameType);

private:
  int32_t initRobots(const RoboCollectorGuiConfig& guiCfg);
  int32_t initPanelHandler(const PanelHandlerConfig& cfg);
  int32_t initCoinHandler(const CoinHandlerConfig& cfg);
  int32_t initController(const RoboCollectorControllerConfig& guiCfg);
  int32_t initTurnHelper(const RoboCollectorGuiConfig& guiCfg);
  int32_t initControllerExternalBridge();

  Image _map;
  PanelHandler _panelHandler;
  Field _field;
  CoinHandler _coinHandler;
  RoboCollectorController _controller;
  TurnHelper _turnHelper;
  GameEndHelper _gameEndHelper;
  CollisionWatcher _collisionWatcher;
  std::array<Robot, Defines::ROBOTS_CTN> _robots;

  std::shared_ptr<ControllerExternalBridge> _controllerExternalBridge;

  Ros2CommunicatorInterface _communicatorOutInterface;

  RoboMinerGui _roboMinerGui;
  RoboCleanerGui _roboCleanerGui;

  GameType _gameType = GameType::COLLECTOR;
};

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUI_H_ */
