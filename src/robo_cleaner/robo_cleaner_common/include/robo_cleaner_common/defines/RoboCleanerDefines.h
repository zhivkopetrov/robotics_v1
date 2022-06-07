#ifndef ROBO_CLEANER_COMMON_ROBOCLEANERDEFINES_H_
#define ROBO_CLEANER_COMMON_ROBOCLEANERDEFINES_H_

//System headers
#include <cstdint>

//Other libraries headers

//Own components headers

//Forward declarations

namespace RoboCleanerDefines {
enum FieldMarkers {
  CHARGING_STATION = '@'
};

enum EnergyDefines {
  MOVES_REGAINED_ON_CHARGE_TURN = 5
};
} //namespace RoboCleanerDefines

bool isRubbishMarker(char marker);
int32_t getRubbishCounter(char marker);

#endif /* ROBO_CLEANER_COMMON_ROBOCLEANERDEFINES_H_ */
