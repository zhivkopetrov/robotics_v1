#ifndef ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERINITHELPER_H_
#define ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERINITHELPER_H_

//System headers
#include <cstdint>
#include <any>

//Other libraries headers
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations
class RoboCollectorController;
struct RoboCollectorControllerLayoutConfig;
struct RoboCollectorControllerLayoutInterface;

class RoboCollectorControllerInitHelper {
public:
  RoboCollectorControllerInitHelper() = delete;

  static ErrorCode init(const std::any &cfg,
                        RoboCollectorController &controller);

private:
  static ErrorCode initLayout(const RoboCollectorControllerLayoutConfig &cfg,
                              RoboCollectorControllerLayoutInterface &interface, //out param
                              RoboCollectorController &controller);

  static ErrorCode initControllerExternalBridge(
      const RoboCollectorControllerLayoutInterface &interface,
      RoboCollectorController &controller);
};

#endif /* ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERINITHELPER_H_ */
