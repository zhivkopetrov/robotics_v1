#ifndef ROBO_COLLECTOR_GUI_FIELDCONFIG_H_
#define ROBO_COLLECTOR_GUI_FIELDCONFIG_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "utils/drawing/Rectangle.h"

//Own components headers
#include "robo_collector_gui/field/config/TileConfig.h"

//Forward declarations

struct FieldConfig {
  Rectangle fieldDimensions;
  uint64_t tileRsrcId = 0;
  uint64_t debugFontRsrcId = 0;
  int32_t tileWidth = 0;
  int32_t tileHeight = 0;
  int32_t rows = 0;
  int32_t cols = 0;
  char emptyTileMarker = '.';
};

#endif /* ROBO_COLLECTOR_GUI_FIELDCONFIG_H_ */

