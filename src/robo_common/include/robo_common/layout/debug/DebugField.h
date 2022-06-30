#ifndef ROBO_COMMON_DEBUGFIELD_H_
#define ROBO_COMMON_DEBUGFIELD_H_

//System headers
#include <cstdint>
#include <string>

//Other libraries headers
#include "manager_utils/drawing/Fbo.h"
#include "manager_utils/drawing/Image.h"
#include "manager_utils/drawing/Text.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "robo_common/layout/debug/config/DebugFieldConfig.h"
#include "robo_common/defines/RoboCommonFunctionalDefines.h"

//Forward declarations

class DebugField {
public:
  ErrorCode init(const DebugFieldConfig &cfg,
                 const FieldDescription &fieldDescr,
                 const GetRobotAbsolutePosCb &getRobotAbsolutePosCb);

  void draw() const;

  void toggleStatus();

  void setMsg(const std::string &msg);

  void process();

private:
  enum class Quadrant {
    TOP_LEFT, TOP_RIGHT, BOTTOM_LEFT, BOTTOM_RIGHT
  };

  void createVisuals(const DebugFieldConfig &cfg);

  void updateFbo();

  Quadrant determineRobotQuadrant() const;
  Quadrant determineFieldQuadrant() const;

  void changeFieldPositionFromQuadrant(Quadrant quadrant);

  GetRobotAbsolutePosCb _getRobotAbsolutePosCb;

  Image _bgrImg;
  Text _msgText;
  Fbo _fbo;

  int32_t _xDelimiter { };
  int32_t _yDelimiter { };
  int32_t _robotWidth { };
  int32_t _robotHeight { };

  bool _isActive = false;
};

#endif /* ROBO_COMMON_DEBUGFIELD_H_ */
