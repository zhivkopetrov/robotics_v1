#ifndef ROBO_MINER_GUI_ROBOMINERGUIINITHELPER_H_
#define ROBO_MINER_GUI_ROBOMINERGUIINITHELPER_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <any>

//Other libraries headers

//Own components headers

//Forward declarations
class RoboMinerGui;
struct RoboMinerLayoutConfig;
struct SolutionValidatorConfig;
struct RoboMinerLayoutInterface;

class RoboMinerGuiInitHelper {
public:
  RoboMinerGuiInitHelper() = delete;

  static int32_t init(const std::any &cfg, RoboMinerGui &gui);

private:
  static int32_t initLayout(const RoboMinerLayoutConfig &cfg,
                            RoboMinerLayoutInterface &interface, //out param
                            RoboMinerGui &gui);

  static int32_t initSolutionValidator(
      const SolutionValidatorConfig &cfg,
      const RoboMinerLayoutInterface &interface, RoboMinerGui &gui);

  static int32_t initControllerExternalBridge(
      const RoboMinerLayoutInterface &interface, RoboMinerGui &gui);
};

#endif /* ROBO_MINER_GUI_ROBOMINERGUIINITHELPER_H_ */
