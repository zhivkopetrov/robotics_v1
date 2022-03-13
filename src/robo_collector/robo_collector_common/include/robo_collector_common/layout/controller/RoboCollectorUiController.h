#ifndef ROBO_COLLECTOR_COMMON_ROBOCOLLECTORUICONTROLLER_H_
#define ROBO_COLLECTOR_COMMON_ROBOCOLLECTORUICONTROLLER_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"

//Own components headers
#include "robo_collector_common/layout/controller/buttons/MoveButton.h"
#include "robo_collector_common/layout/controller/buttons/HelpButton.h"
#include "robo_collector_common/layout/controller/buttons/SettingsButton.h"
#include "robo_collector_common/layout/controller/config/RoboCollectorUiControllerBaseConfig.h"

//Forward declarations
class InputEvent;

struct RoboCollectorUiControllerConfig {
  std::vector<MoveButtonConfig> moveButtonsCfgs;
  SettingsButtonConfig settingsBtnCfg;
  HelpButtonConfig helpBtnCfg;
  uint64_t horDelimiterRsrcId = 0;
  uint64_t vertDelimiterRsrcId = 0;
  LocalControllerMode localControllerMode = LocalControllerMode::DISABLED;
};

struct RoboCollectorUiControllerOutInterface {
  RobotActCb robotActCb;
  HelpActivatedCb helpActivatedCb;
  SettingActivatedCb settingActivatedCb;
};

class RoboCollectorUiController {
public:
  ErrorCode init(const RoboCollectorUiControllerConfig& cfg,
                 const RoboCollectorUiControllerOutInterface& interface);
  void draw() const;
  void handleEvent(const InputEvent& e);
  void onMoveButtonClicked(MoveType moveType);
  void lockInput();
  void unlockInput();
  bool isEnabled() const;
private:
  RobotActCb _robotActCb;
  std::vector<MoveButton> _moveButtons;

  HelpButton _helpButton;
  SettingsButton _settingsButton;

  //TODO animate them
  Image _horDelimiter;
  Image _vertDelimiter;

  LocalControllerMode _mode = LocalControllerMode::DISABLED;
};

#endif /* ROBO_COLLECTOR_COMMON_ROBOCOLLECTORUICONTROLLER_H_ */
