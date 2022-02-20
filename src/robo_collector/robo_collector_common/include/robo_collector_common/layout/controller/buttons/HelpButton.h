#ifndef ROBO_COLLECTOR_COMMON_HELPBUTTON_H_
#define ROBO_COLLECTOR_COMMON_HELPBUTTON_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "manager_utils/input/ButtonBase.h"

//Own components headers
#include "robo_collector_common/defines/RoboCollectorFunctionalDefines.h"

//Forward declarations
class InputEvent;

struct HelpButtonConfig {
  HelpActivatedCb helpActivatedCb;
  uint64_t rsrcId = 0;
};

class HelpButton final : public ButtonBase {
public:
  int32_t init(const HelpButtonConfig& cfg);
  void handleEvent(const InputEvent& e) override;

private:
  HelpActivatedCb _helpActivatedCb;
};

#endif /* ROBO_COLLECTOR_COMMON_HELPBUTTON_H_ */
