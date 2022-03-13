#ifndef ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERLAYOUTINITHELPER_H_
#define ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERLAYOUTINITHELPER_H_

//System headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations
class RoboCollectorControllerLayout;
struct RoboCollectorControllerLayoutOutInterface;
struct RoboCollectorControllerLayoutConfig;
struct RoboCollectorUiControllerBaseConfig;

class RoboCollectorControllerLayoutInitHelper {
public:
  RoboCollectorControllerLayoutInitHelper() = delete;

  static ErrorCode init(
      const RoboCollectorControllerLayoutConfig &cfg,
      const RoboCollectorControllerLayoutOutInterface &outInterface,
      RoboCollectorControllerLayout &layout);

private:
  static ErrorCode initController(
      const RoboCollectorUiControllerBaseConfig& baseCfg,
      const RoboCollectorControllerLayoutOutInterface &outInterface,
      RoboCollectorControllerLayout &layout);
};

#endif /* ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERLAYOUTINITHELPER_H_ */
