#ifndef UR_CONTROL_COMMON_BUTTONHANDLERCONFIG_H_
#define UR_CONTROL_COMMON_BUTTONHANDLERCONFIG_H_

//System headers
#include <cstdint>
#include <string>
#include <vector>

//Other libraries headers
#include "utils/drawing/Point.h"

//Own components headers

//Forward declarations

struct CommandButtonDescription {
  Point pos;
  std::string text;
};

struct ButtonHandlerConfig {
  uint64_t buttonRsrcId { };
  uint64_t buttonFontRsrcId { };
  std::string gripperScriptFolderLocation;
  std::string commandScriptsFolderLocation;
  std::vector<CommandButtonDescription> commandButtonsDescription;
};

#endif /* UR_CONTROL_COMMON_BUTTONHANDLERCONFIG_H_ */
