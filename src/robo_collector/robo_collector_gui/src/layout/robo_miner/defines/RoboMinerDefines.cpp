//Corresponding header
#include "robo_collector_gui/layout/robo_miner/defines/RoboMinerDefines.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/data_type/EnumClassUtils.h"
#include "utils/Log.h"

//Own components headers

char getCrystalMarker(CrystalType type) {
  switch (type) {
  case CrystalType::CYAN:
    return 'c';
  case CrystalType::PURPLE:
    return 'p';
  case CrystalType::BLUE:
    return 'b';
  case CrystalType::GREEN:
    return 'g';
  case CrystalType::RED:
    return 'r';
  default:
    LOGERR("Error, received unsupported CrystalType: %d", getEnumValue(type));
    return '?';
  }
}

CrystalType getCrystalType(char marker) {
  switch (marker) {
  case 'c':
    return CrystalType::CYAN;
  case 'p':
    return CrystalType::PURPLE;
  case 'b':
    return CrystalType::BLUE;
  case 'g':
    return CrystalType::GREEN;
  case 'r':
    return CrystalType::RED;
  default:
    LOGERR("Error, received unsupported crystal marker: %c", marker);
    return CrystalType::RED;
  }
}

