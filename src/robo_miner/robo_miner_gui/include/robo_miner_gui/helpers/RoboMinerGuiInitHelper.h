#ifndef ROBO_MINER_GUI_ROBOMINERGUIINITHELPER_H_
#define ROBO_MINER_GUI_ROBOMINERGUIINITHELPER_H_

//System headers
#include <cstdint>
#include <any>

//Other libraries headers
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations
class RoboMinerGui;
struct RoboMinerLayoutConfig;
struct SolutionValidatorConfig;
struct RoboMinerLayoutInterface;

class RoboMinerGuiInitHelper {
public:
  RoboMinerGuiInitHelper() = delete;

  static ErrorCode init(const std::any &cfg, RoboMinerGui &gui);

private:
  static ErrorCode initLayout(const RoboMinerLayoutConfig &cfg,
                              RoboMinerLayoutInterface &interface, //out param
                              RoboMinerGui &gui);

  static ErrorCode initSolutionValidator(
      const SolutionValidatorConfig &cfg,
      const RoboMinerLayoutInterface &interface, RoboMinerGui &gui);

  static ErrorCode initControllerExternalBridge(
      const RoboMinerLayoutInterface &interface, RoboMinerGui &gui);
};

#endif /* ROBO_MINER_GUI_ROBOMINERGUIINITHELPER_H_ */
