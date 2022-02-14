#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUT_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUT_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <array>

//Other libraries headers
#include "robo_common/layout/field/Field.h"
#include "robo_common/layout/entities/robot/Robot.h"
#include "robo_common/layout/animators/GameEndAnimator.h"

//Own components headers
#include "robo_collector_gui/layout/RoboCollectorLayoutInterfaces.h"
#include "robo_collector_gui/panels/PanelHandler.h"
#include "robo_collector_gui/entities/coin/CoinHandler.h"
#include "robo_collector_gui/controller/RoboCollectorController.h"

#include "robo_collector_gui/robo_miner/RoboMinerGui.h"
#include "robo_collector_gui/robo_cleaner/RoboCleanerGui.h"

//Forward declarations
class InputEvent;
class RoboCollectorLayoutInitHelper;
struct RoboCollectorLayoutConfig;

class RoboCollectorLayout {
public:
  friend class RoboCollectorLayoutInitHelper;

  int32_t init(const RoboCollectorLayoutConfig &cfg,
               const RoboCollectorLayoutOutInterface &interface);
  RoboCollectorLayoutInterface produceInterface();
  void deinit();
  void draw() const;
  void handleEvent(const InputEvent &e);

  //TODO create a separate help page class
  void activateHelpPage();

  void changeGameType(GameType gameType);

private:
  Image _map;
  PanelHandler _panelHandler;
  Field _field;
  CoinHandler _coinHandler;
  RoboCollectorController _controller;
  std::array<Robot, Defines::ROBOTS_CTN> _robots;
  GameEndAnimator _gameEndAnimator;

  RoboMinerGui _roboMinerGui;
  RoboCleanerGui _roboCleanerGui;

  GameType _gameType = GameType::COLLECTOR;
};

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUT_H_ */
