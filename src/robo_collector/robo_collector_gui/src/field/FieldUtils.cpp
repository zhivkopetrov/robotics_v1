//Corresponding header
#include "robo_collector_gui/field/FieldUtils.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/data_type/EnumClassUtils.h"
#include "utils/drawing/Rectangle.h"
#include "utils/Log.h"

//Own components headers

FieldPos FieldUtils::getFieldPos(const Point &absPos) {
  return FieldPos(
      (absPos.y - Defines::FIRST_TILE_X_POS) / Defines::TILE_HEIGHT,
      (absPos.x - Defines::FIRST_TILE_Y_POS) / Defines::TILE_WIDTH);
}

Point FieldUtils::getAbsPos(const FieldPos &boardPos) {
  return Point(Defines::FIRST_TILE_X_POS + boardPos.col * Defines::TILE_WIDTH,
      Defines::FIRST_TILE_Y_POS + boardPos.row * Defines::TILE_HEIGHT);
}

bool FieldUtils::isInsideField(const FieldPos &fieldPos) {
  if (0 > fieldPos.row || Defines::FIELD_ROWS <= fieldPos.row) {
    return false;
  }

  if (0 > fieldPos.col || Defines::FIELD_COLS <= fieldPos.col) {
    return false;
  }

  return true;
}

bool FieldUtils::isInsideField(const Point &absPos) {
  const Rectangle boundary(Defines::FIRST_TILE_X_POS, Defines::FIRST_TILE_Y_POS,
      Defines::FIELD_COLS * Defines::TILE_WIDTH,
      Defines::FIELD_ROWS * Defines::TILE_HEIGHT);

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
    LOGERR("Error, received unknown dir: %d", getEnumValue(dir))
    ;
    break;
  }

  return futurePos;
}

