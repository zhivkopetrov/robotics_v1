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

//Forward declarations
class InputEvent;

struct RoboCollectorControllerConfig {
  RobotActCb robotActCb;
  std::vector<uint64_t> moveButtonsRsrcIds;
  uint64_t moveButtonInfoTextFontId = 0;
  int32_t maxMoveButtons = 0;
};

class RoboCollectorController {
public:
  int32_t init(const RoboCollectorControllerConfig& cfg);
  void draw() const;
  void handleEvent(const InputEvent& e);
  void onMoveButtonClicked(MoveType moveType);
  void unlockInput();
private:
  std::array<MoveButton, Defines::MOVE_BUTTONS_CTN> _moveButtons;
  RobotActCb _robotActCb;
};

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORCONTROLLER_H_ */
