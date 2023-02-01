#ifndef UR_CONTROL_COMMON_SAFETYMODEVISUALS_H_
#define UR_CONTROL_COMMON_SAFETYMODEVISUALS_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "manager_utils/drawing/Text.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "ur_control_common/defines/UrControlCommonDefines.h"

//Forward declarations

class SafetyModeVisuals {
public:
  ErrorCode init(const Rectangle& screenBoundary, uint64_t fontRsrcId);
  void draw() const;

  void changeRobotMode(RobotMode mode);
  void changeSafetyMode(SafetyMode mode);

  Point getUpperLeftBoundaryPos() const;

private:
  RobotMode _robotMode = RobotMode::Unknown;
  Text _robotModeHeaderText;
  Text _robotModeText;

  SafetyMode _safetyMode = SafetyMode::Unknown;
  Text _safetyModeHeaderText;
  Text _safetyModeText;
};

#endif /* UR_CONTROL_COMMON_SAFETYMODEVISUALS_H_ */
