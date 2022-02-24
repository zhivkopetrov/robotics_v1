//Corresponding header
#include "robo_cleaner_gui/defines/RoboCleanerGuiDefines.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "robo_common/defines/RoboCommonDefines.h"
#include "utils/Log.h"

//Own components headers

bool isRubbishMarker(char marker) {
  return (RoboCleanerDefines::SMALL_RUBISH_MARKER == marker) ||
         (RoboCleanerDefines::BIG_RUBBISH_MARKER == marker);
}

int32_t getRubbishCounter(char marker) {
  switch (marker) {
  case RoboCommonDefines::EMPTY_TILE_MARKER:
    return 1;
  case RoboCleanerDefines::SMALL_RUBISH_MARKER:
    return 2;
  case RoboCleanerDefines::BIG_RUBBISH_MARKER:
    return 3;
  default:
    LOGERR("Error, received invalid marker: %c", marker);
    return 0;
  }
}
