#ifndef UR_CONTROL_BLOOM_URCONTROLBLOOMLAYOUTCONFIG_H_
#define UR_CONTROL_BLOOM_URCONTROLBLOOMLAYOUTCONFIG_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "ur_control_common/layout/config/UrControlCommonLayoutConfig.h"

//Own components headers

//Forward declarations

struct UrControlBloomLayoutConfig {
  UrControlCommonLayoutConfig commonLayoutCfg;

  uint64_t roseRsrcId = 0;
  uint64_t jengaRsrcId = 0;
  uint64_t stateVisualsFontRsrcId = 0;
};

#endif /* UR_CONTROL_BLOOM_URCONTROLBLOOMLAYOUTCONFIG_H_ */
