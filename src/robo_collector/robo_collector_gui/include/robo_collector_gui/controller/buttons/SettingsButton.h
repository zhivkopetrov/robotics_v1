#ifndef ROBO_COLLECTOR_GUI_SETTINGSBUTTON_H_
#define ROBO_COLLECTOR_GUI_SETTINGSBUTTON_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "manager_utils/input/ButtonBase.h"

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiFunctionalDefines.h"

//Forward declarations
class InputEvent;

struct SettingsButtonConfig {
  SettingActivatedCb settingActivatedCb;
  uint64_t rsrcId = 0;
};

class SettingsButton final : public ButtonBase {
public:
  int32_t init(const SettingsButtonConfig& cfg);
  void handleEvent(const InputEvent& e) override;

private:
  void changeGameType();

  SettingActivatedCb _settingActivatedCb;
  GameType _currGameType = GameType::COLLECTOR;
};

#endif /* ROBO_COLLECTOR_GUI_SETTINGSBUTTON_H_ */
