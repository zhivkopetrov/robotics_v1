//Corresponding header
#include "robo_cleaner_common/defines/RoboCleanerDefines.h"

//System headers
#include <cctype>

//Other libraries headers
#include "utils/Log.h"

//Own components headers

bool isRubbishMarker(char marker) {
  return std::isdigit(marker);
}

int32_t getRubbishCounter(char marker) {
  return marker - '0';
}


