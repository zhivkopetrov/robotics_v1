#ifndef ROBO_COLLECTOR_COMMON_ROBOCOLLECTORUICONTROLLER_H_
#define ROBO_COLLECTOR_COMMON_ROBOCOLLECTORUICONTROLLER_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <array>

//Other libraries headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"

//Own components headers
#include "robo_collector_common/layout/controller/buttons/MoveButton.h"
#include "robo_collector_common/layout/controller/buttons/HelpButton.h"
#include "robo_collector_common/layout/controller/buttons/SettingsButton.h"
#include "robo_collector_common/layout/controller/config/RoboCollectorUiControllerConfig.h"

//Forward declarations
class InputEvent;

struct RoboCollectorUiControllerOutInterface {
  RobotActCb robotActCb;
  HelpActivatedCb helpActivatedCb;
  SettingActivatedCb settingActivatedCb;
};

class RoboCollectorUiController {
public:
  int32_t init(const RoboCollectorUiControllerConfig& cfg,
               const RoboCollectorUiControllerOutInterface& interface);
  void draw() const;
  void handleEvent(const InputEvent& e);
  void onMoveButtonClicked(MoveType moveType);
  void lockInput();
  void unlockInput();
  bool isEnabled() const;
private:
  enum MoveButtonDefines {
    BUTTON_FORWARD,
    BUTTON_ROTATE_LEFT,
    BUTTON_ROTATE_RIGHT,
    MOVE_BUTTONS_CTN
  };

  RobotActCb _robotActCb;
  std::array<MoveButton, MOVE_BUTTONS_CTN> _moveButtons;

  HelpButton _helpButton;
  SettingsButton _settingsButton;

  //TODO animate them
  Image _horDelimiter;
  Image _vertDelimiter;

  bool _isEnabled = false;
};

#endif /* ROBO_COLLECTOR_COMMON_ROBOCOLLECTORUICONTROLLER_H_ */
