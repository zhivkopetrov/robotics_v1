#ifndef ROBO_CLEANER_GUI_ROBOCLEANERDEFINES_H_
#define ROBO_CLEANER_GUI_ROBOCLEANERDEFINES_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers

//Forward declarations

namespace RoboCleanerDefines {
enum Markers {
  SMALL_RUBISH_MARKER = 'r',
  BIG_RUBBISH_MARKER = 'R',
  OBSTACLE_MARKER = 'x'
};
}

bool isRubbishMarker(char marker);
int32_t getRubbishCounter(char marker);

#endif /* ROBO_CLEANER_GUI_ROBOCLEANERDEFINES_H_ */
