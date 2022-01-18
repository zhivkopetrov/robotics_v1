#ifndef ROBO_COLLECTOR_GUI_FIELDUTILS_H_
#define ROBO_COLLECTOR_GUI_FIELDUTILS_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <array>

//Other libraries headers
#include "utils/drawing/Point.h"

//Own components headers
#include "robo_collector_gui/field/FieldPos.h"
#include "robo_collector_gui/defines/RoboCollectorGuiDefines.h"

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

  static bool doCollideWithEnemy(
      const FieldPos &selectedPos,
      const std::array<Robot, Defines::ENEMIES_CTN> &enemies,
      int32_t *outCollisionRelativeId = nullptr);
};

#endif /* ROBO_COLLECTOR_GUI_FIELDUTILS_H_ */
