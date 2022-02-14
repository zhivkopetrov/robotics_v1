//Corresponding header
#include "robo_common/layout/field/FieldPos.h"

//C system headers

//C++ system headers

//Other libraries headers

//Own components headers

FieldPos::FieldPos(int32_t inputRow, int32_t inputCol) {
  row = inputRow;
  col = inputCol;
}

bool FieldPos::operator==(const FieldPos& other) const {
  return (row == other.row && col == other.col);
}
