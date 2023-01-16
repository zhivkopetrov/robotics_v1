#ifndef UR_CONTROL_GUI_URCONTROLGUILAYOUT_H_
#define UR_CONTROL_GUI_URCONTROLGUILAYOUT_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "manager_utils/drawing/Image.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "ur_control_gui/layout/entities/buttons/ButtonHandler.h"
#include "ur_control_gui/layout/entities/robot/SafetyModeVisuals.h"

//Forward declarations
class InputEvent;
struct UrControlGuiLayoutConfig;
struct UrControlGuiLayoutOutInterface;
struct UrControlGuiLayoutInterface;

class UrControlGuiLayout {
public:
  friend class UrControlGuiLayoutInitHelper;

  ErrorCode init(const UrControlGuiLayoutConfig &cfg,
                 const UrControlGuiLayoutOutInterface& outInterface,
                 UrControlGuiLayoutInterface &interface);
  void deinit();
  void draw() const;
  void handleEvent(const InputEvent &e);

private:
  void produceInterface(UrControlGuiLayoutInterface& interface);

  ButtonHandler _buttonHandler;
  SafetyModeVisuals _safetyModeVisuals;

  Image _map;
  Image _robot;
};

#endif /* UR_CONTROL_GUI_URCONTROLGUILAYOUT_H_ */
