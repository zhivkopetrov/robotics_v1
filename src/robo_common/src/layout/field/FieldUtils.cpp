//Corresponding header
#include "robo_common/layout/field/FieldUtils.h"

//System headers

//Other libraries headers
#include "utils/data_type/EnumClassUtils.h"
#include "utils/drawing/Rectangle.h"
#include "utils/Log.h"

//Own components headers

FieldPos FieldUtils::getFieldPos(const Point &absPos,
                                 const FieldDescription &descr) {
  return FieldPos(
      (absPos.y - RoboCommonDefines::FIRST_TILE_Y_POS) / descr.tileHeight,
      (absPos.x - RoboCommonDefines::FIRST_TILE_X_POS) / descr.tileWidth);
}

Point FieldUtils::getAbsPos(const FieldPos &boardPos,
                            const FieldDescription &descr) {
  return Point(
      RoboCommonDefines::FIRST_TILE_X_POS + (boardPos.col * descr.tileWidth),
      RoboCommonDefines::FIRST_TILE_Y_POS + (boardPos.row * descr.tileHeight));
}

bool FieldUtils::isInsideField(const FieldPos &fieldPos,
                               const FieldDescription &descr) {
  if (0 > fieldPos.row || descr.rows <= fieldPos.row) {
    return false;
  }

  if (0 > fieldPos.col || descr.cols <= fieldPos.col) {
    return false;
  }

  const char tile = descr.data[fieldPos.row][fieldPos.col];
  for (const char obstacleMarker : descr.obstacleMarkers) {
    if (obstacleMarker == tile) {
      return false;
    }
  }

  return true;
}

FieldPos FieldUtils::getAdjacentPos(Direction dir, const FieldPos &fieldPos) {
  FieldPos futurePos = fieldPos;

  switch (dir) {
  case Direction::UP:
    --futurePos.row;
    break;

  case Direction::RIGHT:
    ++futurePos.col;
    break;

  case Direction::DOWN:
    ++futurePos.row;
    break;

  case Direction::LEFT:
    --futurePos.col;
    break;

  default:
    LOGERR("Error, received unknown dir: %d", getEnumValue(dir));
    break;
  }

  return futurePos;
}

