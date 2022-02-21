#ifndef ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERLAYOUTINITHELPER_H_
#define ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERLAYOUTINITHELPER_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>

//Other libraries headers

//Own components headers

//Forward declarations
class RoboCollectorControllerLayout;
struct RoboCollectorControllerLayoutOutInterface;
struct RoboCollectorControllerLayoutConfig;
struct RoboCollectorUiControllerBaseConfig;

class RoboCollectorControllerLayoutInitHelper {
public:
  RoboCollectorControllerLayoutInitHelper() = delete;

  static int32_t init(
      const RoboCollectorControllerLayoutConfig &cfg,
      const RoboCollectorControllerLayoutOutInterface &outInterface,
      RoboCollectorControllerLayout &layout);

private:
  static int32_t initController(
      const RoboCollectorUiControllerBaseConfig& baseCfg,
      const RoboCollectorControllerLayoutOutInterface &outInterface,
      RoboCollectorControllerLayout &layout);
};

#endif /* ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERLAYOUTINITHELPER_H_ */
