#ifndef UR_CONTROL_GUI_URCONTROLGUILAYOUTCONFIG_H_
#define UR_CONTROL_GUI_URCONTROLGUILAYOUTCONFIG_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "utils/drawing/Rectangle.h"

//Own components headers
#include "ur_control_gui/layout/entities/buttons/config/ButtonHandlerConfig.h"

//Forward declarations

struct UrControlGuiLayoutConfig {
  ButtonHandlerConfig buttonHandlerConfig;

  Rectangle screenBoundary;
  uint64_t mapRsrcId = 0;
  uint64_t robotImgRrscId = 0;
};

#endif /* UR_CONTROL_GUI_URCONTROLGUILAYOUTCONFIG_H_ */
