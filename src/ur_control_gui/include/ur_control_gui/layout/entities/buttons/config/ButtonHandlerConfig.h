#ifndef UR_CONTROL_GUI_BUTTONHANDLERCONFIG_H_
#define UR_CONTROL_GUI_BUTTONHANDLERCONFIG_H_

//System headers
#include <cstdint>
#include <string>

//Other libraries headers

//Own components headers

//Forward declarations

struct ButtonHandlerConfig {
  uint64_t buttonRsrcId { };
  uint64_t buttonFontRsrcId { };
  std::string scriptFolderLocation;
};

#endif /* UR_CONTROL_GUI_BUTTONHANDLERCONFIG_H_ */
