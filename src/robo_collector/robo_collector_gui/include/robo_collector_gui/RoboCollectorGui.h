#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUI_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUI_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <array>

//Other libraries headers
#include "game_engine/Game.h"

//Own components headers
#include "robo_collector_gui/field/Field.h"
#include "robo_collector_gui/entities/Robot.h"
#include "robo_collector_gui/entities/coin/Coin.h"
#include "robo_collector_gui/panels/Panel.h"
#include "robo_collector_gui/buttons/MoveButton.h"

//Forward declarations
class InputEvent;

class RoboCollectorGui final : public Game {
public:
  int32_t init(const std::any& cfg) override;
  void deinit() override;

  void draw() const override;
  void handleEvent(const InputEvent &e) override;

private:
  enum InternalDefines {
    ENEMIES_CTN = 3,
    MOVE_BUTTONS_CTN = 3
  };

  Robot _blinky;
  std::array<Robot, ENEMIES_CTN> _enemies;
  std::array<MoveButton, MOVE_BUTTONS_CTN> _moveButtons;
  Panel _panel;
  Coin _coin;
  Field _field;
};

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUI_H_ */
