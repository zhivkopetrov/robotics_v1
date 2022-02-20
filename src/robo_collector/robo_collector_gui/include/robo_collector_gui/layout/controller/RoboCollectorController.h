#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORCONTROLLER_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORCONTROLLER_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <array>

//Other libraries headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiDefines.h"
#include "robo_collector_gui/layout/controller/buttons/MoveButton.h"
#include "robo_collector_gui/layout/controller/buttons/HelpButton.h"
#include "robo_collector_gui/layout/controller/buttons/SettingsButton.h"
#include "robo_collector_gui/layout/controller/config/RoboCollectorControllerConfig.h"

//Forward declarations
class InputEvent;

struct RoboCollectorControllerOutInterface {
  RobotActCb robotActCb;
  HelpActivatedCb helpActivatedCb;
  SettingActivatedCb settingActivatedCb;
};

class RoboCollectorController {
public:
  int32_t init(const RoboCollectorControllerConfig& cfg,
               const RoboCollectorControllerOutInterface& interface);
  void draw() const;
  void handleEvent(const InputEvent& e);
  void onMoveButtonClicked(MoveType moveType);
  void lockInput();
  void unlockInput();
  bool isEnabled() const;
private:
  RobotActCb _robotActCb;
  std::array<MoveButton, Defines::MOVE_BUTTONS_CTN> _moveButtons;

  HelpButton _helpButton;
  SettingsButton _settingsButton;

  //TODO animate them
  Image _horDelimiter;
  Image _vertDelimiter;

  bool _isEnabled = false;
};

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORCONTROLLER_H_ */
