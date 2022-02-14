//Corresponding header
#include "robo_common/layout/field/FieldUtils.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/data_type/EnumClassUtils.h"
#include "utils/drawing/Rectangle.h"
#include "utils/Log.h"

//Own components headers

FieldPos FieldUtils::getFieldPos(const Point &absPos) {
  return FieldPos(
      (absPos.y - RoboCommonDefines::FIRST_TILE_Y_POS) /
        RoboCommonDefines::TILE_HEIGHT,
      (absPos.x - RoboCommonDefines::FIRST_TILE_X_POS) /
        RoboCommonDefines::TILE_WIDTH);
}

Point FieldUtils::getAbsPos(const FieldPos &boardPos) {
  return Point(
      RoboCommonDefines::FIRST_TILE_X_POS +
        (boardPos.col * RoboCommonDefines::TILE_WIDTH),
      RoboCommonDefines::FIRST_TILE_Y_POS +
        (boardPos.row * RoboCommonDefines::TILE_HEIGHT));
}

bool FieldUtils::isInsideField(const FieldPos &fieldPos) {
  if (0 > fieldPos.row || RoboCommonDefines::FIELD_ROWS <= fieldPos.row) {
    return false;
  }

  if (0 > fieldPos.col || RoboCommonDefines::FIELD_COLS <= fieldPos.col) {
    return false;
  }

  return true;
}

bool FieldUtils::isInsideField(const Point &absPos) {
  const Rectangle boundary(RoboCommonDefines::FIRST_TILE_X_POS,
      RoboCommonDefines::FIRST_TILE_Y_POS,
      RoboCommonDefines::FIELD_COLS * RoboCommonDefines::TILE_WIDTH,
      RoboCommonDefines::FIELD_ROWS * RoboCommonDefines::TILE_HEIGHT);

  return boundary.isPointInRect(absPos);
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

