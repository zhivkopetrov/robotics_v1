#ifndef ROBO_COMMON_DEBUGFIELDCONFIG_H_
#define ROBO_COMMON_DEBUGFIELDCONFIG_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "utils/drawing/Rectangle.h"

//Own components headers

//Forward declarations

struct DebugFieldConfig {
  Rectangle dimensions = Rectangle(0, 0, 480, 60);
  uint64_t panelRsrcId { };
  uint64_t texFotnRsrcId { };
};

#endif /* ROBO_COMMON_DEBUGFIELDCONFIG_H_ */
