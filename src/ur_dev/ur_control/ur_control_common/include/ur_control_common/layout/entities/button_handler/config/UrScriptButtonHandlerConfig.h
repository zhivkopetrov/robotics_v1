#ifndef UR_CONTROL_COMMON_URSCRIPTBUTTONHANDLERCONFIG_H_
#define UR_CONTROL_COMMON_URSCRIPTBUTTONHANDLERCONFIG_H_

//System headers
#include <cstdint>
#include <string>
#include <vector>

//Other libraries headers

//Own components headers
#include "ur_control_common/layout/entities/button_handler/config/ButtonHandlerConfig.h"

//Forward declarations

struct UrScriptButtonHandlerConfig {
  ButtonHandlerConfig baseCfg;
  std::string gripperScriptFolderLocation;
  std::string commandScriptsFolderLocation;
  std::vector<CommandButtonDescription> commandButtonsDescription;
};

#endif /* UR_CONTROL_COMMON_URSCRIPTBUTTONHANDLERCONFIG_H_ */
