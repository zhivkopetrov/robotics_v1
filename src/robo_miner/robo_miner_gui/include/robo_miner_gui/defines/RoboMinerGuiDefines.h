#ifndef ROBO_MINER_GUI_ROBOMINERDEFINES_H_
#define ROBO_MINER_GUI_ROBOMINERDEFINES_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers

//Forward declarations

enum class CrystalType {
  CYAN,
  PURPLE,
  BLUE,
  GREEN,
  RED
};

char getCrystalMarker(CrystalType type);
CrystalType getCrystalType(char marker);

#endif /* ROBO_MINER_GUI_ROBOMINERDEFINES_H_ */
