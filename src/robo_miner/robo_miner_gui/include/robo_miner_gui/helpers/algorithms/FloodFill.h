#ifndef ROBO_MINER_GUI_FLOODFILL_H_
#define ROBO_MINER_GUI_FLOODFILL_H_

//System headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "robo_common/defines/RoboCommonDefines.h"
#include "robo_common/layout/field/FieldPos.h"

//Own components headers

//Forward declarations

class FloodFill {
public:
  FloodFill() = delete;

  static std::vector<FieldPos> findLongestCrystalSequence(
      const FieldData &data, const std::vector<char> &nonCrystalMarkers);

  static std::vector<FieldPos> findLocalCrystalSequence(
      const FieldData &data, const std::vector<char> &nonCrystalMarkers,
      const FieldPos &fieldPos);
};

#endif /* ROBO_MINER_GUI_FLOODFILL_H_ */
