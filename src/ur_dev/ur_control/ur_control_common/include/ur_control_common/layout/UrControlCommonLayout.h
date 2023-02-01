#ifndef UR_CONTROL_COMMON_URCONTROLCOMMONLAYOUT_H_
#define UR_CONTROL_COMMON_URCONTROLCOMMONLAYOUT_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "manager_utils/drawing/Image.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "ur_control_common/layout/entities/buttons/ButtonHandler.h"
#include "ur_control_common/layout/entities/robot/SafetyModeVisuals.h"

//Forward declarations
class InputEvent;
struct UrControlCommonLayoutConfig;
struct UrControlCommonLayoutOutInterface;
struct UrControlCommonLayoutInterface;

class UrControlCommonLayout {
public:
  friend class UrControlCommonLayoutInitHelper;

  ErrorCode init(const UrControlCommonLayoutConfig &cfg,
                 const UrControlCommonLayoutOutInterface& outInterface,
                 UrControlCommonLayoutInterface &interface);
  void deinit();
  void draw() const;
  void handleEvent(const InputEvent &e);

protected:
  ButtonHandler buttonHandler;
  SafetyModeVisuals safetyModeVisuals;

private:
  void produceInterface(UrControlCommonLayoutInterface& interface);

  Image _map;
  Image _robot;
};

#endif /* UR_CONTROL_COMMON_URCONTROLCOMMONLAYOUT_H_ */
