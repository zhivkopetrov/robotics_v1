#ifndef ROBO_COMMON_ROBOTACTINTERFACE_H_
#define ROBO_COMMON_ROBOTACTINTERFACE_H_

//System headers
#include <cstdint>

//Other libraries headers

//Own components headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"

//Forward declarations

struct RobotActInterface {
  RobotActInterface() = default;

  RobotActInterface(const RobotActCb &inputActCb,
                    const GetRobotStateCb &inputGetRobotStateCb)
      : actCb(inputActCb), getRobotStateCb(inputGetRobotStateCb) {
  }

  RobotActCb actCb;
  GetRobotStateCb getRobotStateCb;
};

#endif /* ROBO_COMMON_ROBOTACTINTERFACE_H_ */
