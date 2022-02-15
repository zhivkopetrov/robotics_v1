#ifndef ROBO_MINER_GUI_FLOODFILL_H_
#define ROBO_MINER_GUI_FLOODFILL_H_

//C system headers

//C++ system headers
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
      const FieldData &data, char emptyMarker);

  static std::vector<FieldPos> findLocalCrystalSequence(
      const FieldData &data, const FieldPos& fieldPos, char emptyMarker);
};

#endif /* ROBO_MINER_GUI_FLOODFILL_H_ */
