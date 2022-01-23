#ifndef ROBO_COLLECTOR_GUI_MOVEBUTTON_H_
#define ROBO_COLLECTOR_GUI_MOVEBUTTON_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <string>
#include <functional>

//Other libraries headers
#include "manager_utils/input/ButtonBase.h"
#include "manager_utils/drawing/Text.h"

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiDefines.h"
#include "robo_collector_gui/defines/RoboCollectorGuiFunctionalDefines.h"

//Forward declarations
class InputEvent;

struct MoveButtonCfg {
  RobotActCb robotActCb;
  Point startPos;
  uint64_t rsrcId = 0;
  MoveType moveType = MoveType::UNKNOWN;
  uint64_t infoTextFontId = 0;
  std::string infoTextContent;
  Point infoTextPos;
};

class MoveButton final : public ButtonBase {
public:
  int32_t init(const MoveButtonCfg& cfg);
  void handleEvent(const InputEvent& e) override;
  void draw() const;

private:
  RobotActCb _robotActCb;
  MoveType _moveType = MoveType::UNKNOWN;
  Text _infoText;
};

#endif /* ROBO_COLLECTOR_GUI_MOVEBUTTON_H_ */
