#ifndef ROBO_COLLECTOR_COMMON_MOVEBUTTON_H_
#define ROBO_COLLECTOR_COMMON_MOVEBUTTON_H_

//System headers
#include <cstdint>
#include <string>

//Other libraries headers
#include "manager_utils/input/ButtonBase.h"
#include "manager_utils/drawing/Text.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "robo_collector_common/defines/RoboCollectorFunctionalDefines.h"

//Forward declarations
class InputEvent;

struct MoveButtonConfig {
  Point startPos;
  uint64_t rsrcId = 0;
  MoveType moveType = MoveType::UNKNOWN;
  uint64_t infoTextFontId = 0;
  std::string infoTextContent;
  Point infoTextPos;
};

class MoveButton final : public ButtonBase {
public:
  ErrorCode init(const MoveButtonConfig& cfg, const MoveButtonClickCb& clickCb);
  void handleEvent(const InputEvent& e) override;
  void draw() const;

private:
  MoveButtonClickCb _clickCb;
  MoveType _moveType = MoveType::UNKNOWN;
  Text _infoText;
};

#endif /* ROBO_COLLECTOR_COMMON_MOVEBUTTON_H_ */
