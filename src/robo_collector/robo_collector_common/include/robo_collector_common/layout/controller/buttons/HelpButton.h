#ifndef ROBO_COLLECTOR_COMMON_HELPBUTTON_H_
#define ROBO_COLLECTOR_COMMON_HELPBUTTON_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "manager_utils/input/ButtonBase.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "robo_collector_common/defines/RoboCollectorFunctionalDefines.h"

//Forward declarations
class InputEvent;

struct HelpButtonConfig {
  uint64_t rsrcId = 0;
  Point pos;
};

class HelpButton final : public ButtonBase {
public:
  ErrorCode init(const HelpButtonConfig& cfg,
                 const HelpActivatedCb& helpActivatedCb);
  void handleEvent(const InputEvent& e) override;

private:
  HelpActivatedCb _helpActivatedCb;
};

#endif /* ROBO_COLLECTOR_COMMON_HELPBUTTON_H_ */
