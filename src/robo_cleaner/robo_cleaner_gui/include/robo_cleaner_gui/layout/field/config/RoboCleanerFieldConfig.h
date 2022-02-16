#ifndef ROBO_CLEANER_GUI_ROBOCLEANERFIELDCONFIG_H_
#define ROBO_CLEANER_GUI_ROBOCLEANERFIELDCONFIG_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers

//Forward declarations

struct RoboCleanerFieldConfig {
  uint64_t rubbishRsrcId = 0;
  uint64_t rubbishFontId = 0;
  uint64_t obstacleRsrcId = 0;

  int32_t rows = 0;
  int32_t cols = 0;
  char emptyTileMarker = '.';
};

#endif /* ROBO_CLEANER_GUI_ROBOCLEANERFIELDCONFIG_H_ */
