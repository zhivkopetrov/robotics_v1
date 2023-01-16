#ifndef UR_CONTROL_GUI_COMMANDBUTTON_H_
#define UR_CONTROL_GUI_COMMANDBUTTON_H_

//System headers
#include <cstdint>
#include <string>

//Other libraries headers
#include "manager_utils/input/ButtonBase.h"
#include "manager_utils/drawing/Text.h"
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations
class InputEvent;

struct CommandButtonConfig {
  uint64_t rsrcId { };
  uint64_t fontRsrcId { };
  Point pos;
  Color descriptionColor = Colors::BLACK;
  std::string descriptionText;
  int32_t descriptionOffsetY { }; //transfer text below button
};

class CommandButton : public ButtonBase {
public:
  ErrorCode init(const CommandButtonConfig &cfg);
  void draw() const;

protected:
  Text _description;
};

#endif /* UR_CONTROL_GUI_COMMANDBUTTON_H_ */
