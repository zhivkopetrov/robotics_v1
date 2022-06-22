#ifndef ROBO_COLLECTOR_CONTROLLER_USERAUTHENTICATEHELPERCONFIG_H_
#define ROBO_COLLECTOR_CONTROLLER_USERAUTHENTICATEHELPERCONFIG_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "robo_common/defines/RoboCommonDefines.h"

//Own components headers

//Forward declarations

struct UserAuthenticateHelperConfig {
  UserData userData;
  int32_t timerId { };
};

#endif /* ROBO_COLLECTOR_CONTROLLER_USERAUTHENTICATEHELPERCONFIG_H_ */
