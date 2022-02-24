#ifndef ROBO_COMMON_FIELDCONFIG_H_
#define ROBO_COMMON_FIELDCONFIG_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers
#include "robo_common/defines/RoboCommonDefines.h"

//Forward declarations

struct FieldConfig {
  FieldDescription description;
  uint64_t tileRsrcId = 0;
  uint64_t debugFontRsrcId = 0;
};

#endif /* ROBO_COMMON_FIELDCONFIG_H_ */

