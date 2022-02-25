#ifndef ROBO_COMMON_ROBOTINITHELPER_H_
#define ROBO_COMMON_ROBOTINITHELPER_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers

//Forward declarations
class Robot;
struct RobotState;
struct RobotAnimatorConfigBase;
struct RobotOutInterface;

class RobotInitHelper {
public:
  RobotInitHelper() = delete;

  static int32_t init(const RobotState &initialState,
                      const RobotAnimatorConfigBase &robotAnimCfgBase,
                      const RobotOutInterface &interface, Robot &robot);

private:
  static int32_t initOutInterface(const RobotOutInterface &interface,
                                  Robot &robot);
  static int32_t initAnimator(
      const RobotAnimatorConfigBase &robotAnimCfgBase, Robot &robot);
};

#endif /* ROBO_COMMON_ROBOTINITHELPER_H_ */
