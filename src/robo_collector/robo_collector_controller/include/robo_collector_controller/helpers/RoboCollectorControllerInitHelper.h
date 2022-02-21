#ifndef ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERINITHELPER_H_
#define ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERINITHELPER_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <any>

//Other libraries headers

//Own components headers

//Forward declarations
class RoboCollectorController;
struct RoboCollectorControllerLayoutConfig;
struct RoboCollectorControllerLayoutInterface;

class RoboCollectorControllerInitHelper {
public:
  RoboCollectorControllerInitHelper() = delete;

  static int32_t init(const std::any &cfg, RoboCollectorController &controller);

private:
  static int32_t initLayout(
      const RoboCollectorControllerLayoutConfig &cfg,
      RoboCollectorControllerLayoutInterface &interface, //out param
      RoboCollectorController &controller);

  static int32_t initControllerExternalBridge(
      const RoboCollectorControllerLayoutInterface &interface,
      RoboCollectorController &controller);
};

#endif /* ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERINITHELPER_H_ */
