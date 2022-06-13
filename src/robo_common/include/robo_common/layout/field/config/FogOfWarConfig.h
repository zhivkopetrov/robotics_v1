#ifndef ROBO_COMMON_FOGOFWARCONFIG_H_
#define ROBO_COMMON_FOGOFWARCONFIG_H_

//System headers
#include <cstdint>
#include <vector>

//Other libraries headers

//Own components headers
#include "robo_common/layout/field/FieldPos.h"

//Forward declarations

enum class FogOfWarStatus {
  ENABLED, DISABLED
};

struct FogOfWarConfig {
  FieldPos playerStartingPos;
  std::vector<int> fogTilesFadeAnimTimerIds; //size == number of field tiles
  uint64_t cloudRsrcId = 0;
  FogOfWarStatus status = FogOfWarStatus::DISABLED;
};

#endif /* ROBO_COMMON_FOGOFWARCONFIG_H_ */

