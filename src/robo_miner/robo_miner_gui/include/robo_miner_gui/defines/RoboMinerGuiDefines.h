#ifndef ROBO_MINER_GUI_ROBOMINERDEFINES_H_
#define ROBO_MINER_GUI_ROBOMINERDEFINES_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "robo_common/layout/field/FieldPos.h"

//Own components headers

//Forward declarations

using CrystalSequence = std::vector<FieldPos>;

enum class CrystalType {
  CYAN,
  PURPLE,
  BLUE,
  GREEN,
  RED
};

char getCrystalMarker(CrystalType type);
CrystalType getCrystalType(char marker);
bool isCrystalMarker(char marker);

#endif /* ROBO_MINER_GUI_ROBOMINERDEFINES_H_ */
