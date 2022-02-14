#ifndef ROBO_COMMON_FIELDUTILS_H_
#define ROBO_COMMON_FIELDUTILS_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <array>

//Other libraries headers
#include "utils/drawing/Point.h"

//Own components headers
#include "robo_common/defines/RoboCommonDefines.h"
#include "robo_common/layout/field/FieldPos.h"

//Forward declarations
class Robot;

class FieldUtils {
public:
  FieldUtils() = delete;

  static FieldPos getFieldPos(const Point &absPos);

  static Point getAbsPos(const FieldPos &fieldPos);

  static bool isInsideField(const FieldPos &fieldPos);

  static bool isInsideField(const Point &absPos);

  static FieldPos getAdjacentPos(Direction dir, const FieldPos &fieldPos);
};

#endif /* ROBO_COMMON_FIELDUTILS_H_ */
