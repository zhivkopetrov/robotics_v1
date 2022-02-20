#ifndef ROBO_COLLECTOR_COMMON_MOVEBUTTON_H_
#define ROBO_COLLECTOR_COMMON_MOVEBUTTON_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <string>

//Other libraries headers
#include "manager_utils/input/ButtonBase.h"
#include "manager_utils/drawing/Text.h"

//Own components headers
#include "robo_collector_common/defines/RoboCollectorFunctionalDefines.h"

//Forward declarations
class InputEvent;

struct MoveButtonConfig {
  MoveButtonClickCb clickCb;
  Point startPos;
  uint64_t rsrcId = 0;
  MoveType moveType = MoveType::UNKNOWN;
  uint64_t infoTextFontId = 0;
  std::string infoTextContent;
  Point infoTextPos;
};

class MoveButton final : public ButtonBase {
public:
  int32_t init(const MoveButtonConfig& cfg);
  void handleEvent(const InputEvent& e) override;
  void draw() const;

private:
  MoveButtonClickCb _clickCb;
  MoveType _moveType = MoveType::UNKNOWN;
  Text _infoText;
};

#endif /* ROBO_COLLECTOR_COMMON_MOVEBUTTON_H_ */
