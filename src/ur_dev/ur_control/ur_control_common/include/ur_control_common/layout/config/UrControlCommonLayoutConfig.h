#ifndef UR_CONTROL_COMMON_URCONTROLCOMMONLAYOUTCONFIG_H_
#define UR_CONTROL_COMMON_URCONTROLCOMMONLAYOUTCONFIG_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "utils/drawing/Rectangle.h"

//Own components headers
#include "ur_control_common/layout/entities/button_handler/config/ButtonHandlerConfig.h"

//Forward declarations

struct UrControlCommonLayoutConfig {
  ButtonHandlerHighLevelConfig buttonHandlerHighLevelCfg;

  Rectangle screenBoundary;
  uint64_t mapRsrcId = 0;
  uint64_t robotImgRrscId = 0;
  uint64_t robotModeVisualsFontRsrcId = 0;
};

#endif /* UR_CONTROL_COMMON_URCONTROLCOMMONLAYOUTCONFIG_H_ */
