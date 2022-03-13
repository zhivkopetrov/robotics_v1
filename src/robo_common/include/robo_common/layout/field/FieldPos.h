#ifndef ROBO_COMMON_FIELDPOS_H_
#define ROBO_COMMON_FIELDPOS_H_

//System headers
#include <cstdint>

//Other libraries headers

//Own components headers

//Forward declarations

struct FieldPos {
  FieldPos() = default;
  FieldPos(int32_t row, int32_t col);

  bool operator==(const FieldPos& other) const;
  bool operator<(const FieldPos& other) const;

  int32_t row { 0 };
  int32_t col { 0 };
};

#endif /* ROBO_COMMON_FIELDPOS_H_ */
