#ifndef ROBO_COMMON_ROBOTACTINTERFACE_H_
#define ROBO_COMMON_ROBOTACTINTERFACE_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"

//Forward declarations

struct RobotActInterface {
  RobotActInterface() = default;

  RobotActInterface(const RobotActCb inputActCb,
                    GetRobotFieldPosCb inputGetFieldPosCb,
                    GetRobotDirCb inputGetDirCb)
      : actCb(inputActCb), getFieldPosCb(inputGetFieldPosCb),
        getDirCb(inputGetDirCb) {
  }

  RobotActCb actCb;
  GetRobotFieldPosCb getFieldPosCb;
  GetRobotDirCb getDirCb;
};

#endif /* ROBO_COMMON_ROBOTACTINTERFACE_H_ */
