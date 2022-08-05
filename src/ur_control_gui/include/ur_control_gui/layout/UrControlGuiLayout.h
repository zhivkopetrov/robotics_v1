#ifndef UR_CONTROL_GUI_URCONTROLGUILAYOUT_H_
#define UR_CONTROL_GUI_URCONTROLGUILAYOUT_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "manager_utils/drawing/Image.h"
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations
class InputEvent;
struct UrControlGuiLayoutConfig;
struct UrControlGuiLayoutOutInterface;

class UrControlGuiLayout {
public:
  friend class UrControlGuiLayoutInitHelper;

  ErrorCode init(const UrControlGuiLayoutConfig &cfg,
                 const UrControlGuiLayoutOutInterface& outInterface);
  void deinit();
  void draw() const;
  void handleEvent(const InputEvent &e);

private:
  Image _map;
};

#endif /* UR_CONTROL_GUI_URCONTROLGUILAYOUT_H_ */
