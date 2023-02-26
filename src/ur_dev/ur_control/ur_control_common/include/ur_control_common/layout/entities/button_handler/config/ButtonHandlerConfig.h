#ifndef UR_CONTROL_COMMON_BUTTONHANDLERCONFIG_H_
#define UR_CONTROL_COMMON_BUTTONHANDLERCONFIG_H_

//System headers
#include <cstdint>
#include <string>
#include <any>

//Other libraries headers
#include "utils/drawing/Point.h"

//Own components headers

//Forward declarations

enum ButtonHandlerType {
  URSCRIPT,
  CUSTOM_ACTION
};

struct CommandButtonDescription {
  Point pos;
  std::string text;
};

struct ButtonHandlerConfig {
  uint64_t buttonRsrcId { };
  uint64_t buttonFontRsrcId { };
};

struct ButtonHandlerHighLevelConfig {
  ButtonHandlerType type = ButtonHandlerType::URSCRIPT;
  std::any cfg;
};

#endif /* UR_CONTROL_COMMON_BUTTONHANDLERCONFIG_H_ */
