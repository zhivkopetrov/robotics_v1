#ifndef ROBO_CLEANER_GUI_ENTITYHANDLERCONFIG_H_
#define ROBO_CLEANER_GUI_ENTITYHANDLERCONFIG_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "robo_common/layout/field/FieldPos.h"

//Own components headers

//Forward declarations

struct EntityHandlerConfig {
  uint64_t rubbishRsrcId = 0;
  uint64_t rubbishFontId = 0;
  uint64_t chargingStationRsrcId = 0;
  FieldPos playerStartPosition;
};

#endif /* ROBO_CLEANER_GUI_ENTITYHANDLERCONFIG_H_ */
