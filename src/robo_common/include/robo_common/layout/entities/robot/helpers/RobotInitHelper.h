#ifndef ROBO_COMMON_ROBOTINITHELPER_H_
#define ROBO_COMMON_ROBOTINITHELPER_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations
class Robot;
struct RobotState;
struct RobotAnimatorConfigBase;
struct RobotOutInterface;

class RobotInitHelper {
public:
  RobotInitHelper() = delete;

  static ErrorCode init(const RobotState &initialState,
                        const RobotAnimatorConfigBase &robotAnimCfgBase,
                        const RobotOutInterface &interface, Robot &robot);

private:
  static ErrorCode initOutInterface(const RobotOutInterface &interface,
                                    Robot &robot);
  static ErrorCode initAnimator(const RobotAnimatorConfigBase &robotAnimCfgBase,
                                Robot &robot);
};

#endif /* ROBO_COMMON_ROBOTINITHELPER_H_ */
