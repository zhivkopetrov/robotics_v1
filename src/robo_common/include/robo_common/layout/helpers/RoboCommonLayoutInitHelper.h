#ifndef ROBO_COMMON_ROBOCOMMONLAYOUTINITHELPER_H_
#define ROBO_COMMON_ROBOCOMMONLAYOUTINITHELPER_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers

//Forward declarations
class RoboCommonLayout;
struct RoboCommonLayoutInterface;
struct RoboCommonLayoutOutInterface;
struct RoboCommonLayoutConfig;

class RoboCommonLayoutInitHelper {
public:
  RoboCommonLayoutInitHelper() = delete;

  static int32_t init(const RoboCommonLayoutConfig &cfg,
                      const RoboCommonLayoutOutInterface &outInterface,
                      RoboCommonLayout &layout);

private:
  static int32_t initPlayerRobot(
      const RoboCommonLayoutConfig &layoutCfg,
      const RoboCommonLayoutOutInterface &outInterface,
      RoboCommonLayout &layout);
};

#endif /* ROBO_COMMON_ROBOCOMMONLAYOUTINITHELPER_H_ */
