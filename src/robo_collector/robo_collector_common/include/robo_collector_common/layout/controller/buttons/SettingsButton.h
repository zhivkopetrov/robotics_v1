#ifndef ROBO_COLLECTOR_COMMON_SETTINGSBUTTON_H_
#define ROBO_COLLECTOR_COMMON_SETTINGSBUTTON_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "manager_utils/input/ButtonBase.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "robo_collector_common/defines/RoboCollectorFunctionalDefines.h"

//Forward declarations
class InputEvent;

struct SettingsButtonConfig {
  uint64_t rsrcId = 0;
  Point pos;
};

class SettingsButton final : public ButtonBase {
public:
  ErrorCode init(const SettingsButtonConfig& cfg,
                 const SettingActivatedCb& settingActivatedCb);
  void handleEvent(const InputEvent& e) override;

private:
  SettingActivatedCb _settingActivatedCb;
};

#endif /* ROBO_COLLECTOR_COMMON_SETTINGSBUTTON_H_ */
