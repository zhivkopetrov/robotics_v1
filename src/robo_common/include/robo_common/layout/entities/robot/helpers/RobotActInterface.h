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
                    const GetRobotStateCb &inputGetRobotStateCb,
                    const CancelRobotMove &inputCancelRobotMove,
                    const GetRobotAbsolutePosCb &inputGetRobotAbsolutePosCb,
                    const GetRobotRotationAngleCb &getRobotRotationAngleCb)
      : actCb(inputActCb), getRobotStateCb(inputGetRobotStateCb),
        cancelRobotMove(inputCancelRobotMove),
        getRobotAbsolutePosCb(inputGetRobotAbsolutePosCb),
        getRobotRotationAngleCb(getRobotRotationAngleCb) {
  }

  bool isValid() const {
    return actCb && getRobotStateCb && cancelRobotMove && getRobotAbsolutePosCb
           && getRobotRotationAngleCb;
  }

  RobotActCb actCb;
  GetRobotStateCb getRobotStateCb;
  CancelRobotMove cancelRobotMove;
  GetRobotAbsolutePosCb getRobotAbsolutePosCb;
  GetRobotRotationAngleCb getRobotRotationAngleCb;
};

#endif /* ROBO_COMMON_ROBOTACTINTERFACE_H_ */
