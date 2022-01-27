#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORCONTROLLER_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORCONTROLLER_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <array>

//Other libraries headers

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiDefines.h"
#include "robo_collector_gui/controller/MoveButton.h"
#include "robo_collector_gui/controller/config/RoboCollectorControllerConfig.h"

//Forward declarations
class InputEvent;

struct RoboCollectorControllerOutInterface {
  RobotActCb robotActCb;
};

class RoboCollectorController {
public:
  int32_t init(const RoboCollectorControllerConfig& cfg,
               const RoboCollectorControllerOutInterface& interface);
  void draw() const;
  void handleEvent(const InputEvent& e);
  void onMoveButtonClicked(MoveType moveType);
  void unlockInput();
private:
  RoboCollectorControllerOutInterface _outInterface;
  std::array<MoveButton, Defines::MOVE_BUTTONS_CTN> _moveButtons;

  //TODO animate them
  Image _horDelimiter;
  Image _vertDelimiter;
};

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORCONTROLLER_H_ */
