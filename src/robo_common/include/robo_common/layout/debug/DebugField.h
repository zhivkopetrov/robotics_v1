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
  void updateFbo();

  GetRobotAbsolutePosCb _getRobotAbsolutePosCb;

  Image _bgrImg;
  Text _msgText;
  Fbo _fbo;

  int32_t _xDelimiter { };
  int32_t _yDelimiter { };

  bool _isActive = false;
};

#endif /* ROBO_COMMON_DEBUGFIELD_H_ */
