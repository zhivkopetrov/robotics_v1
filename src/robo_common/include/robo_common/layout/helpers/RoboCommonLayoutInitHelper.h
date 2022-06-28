#ifndef ROBO_COMMON_ROBOCOMMONLAYOUTINITHELPER_H_
#define ROBO_COMMON_ROBOCOMMONLAYOUTINITHELPER_H_

//System headers
#include <cstdint>

//Other libraries headers

//Own components headers
#include "utils/ErrorCode.h"

//Forward declarations
class RoboCommonLayout;
struct RoboCommonLayoutInterface;
struct RoboCommonLayoutOutInterface;
struct RoboCommonLayoutConfig;
struct FieldConfig;
struct GameEndAnimatorConfig;

class RoboCommonLayoutInitHelper {
public:
  RoboCommonLayoutInitHelper() = delete;

  static ErrorCode init(const RoboCommonLayoutConfig &cfg,
                        const RoboCommonLayoutOutInterface &outInterface,
                        RoboCommonLayout &layout);

private:
  static ErrorCode initField(const FieldConfig &cfg,
                             const RoboCommonLayoutOutInterface &outInterface,
                             RoboCommonLayout &layout);

  static ErrorCode initGameEndAnimator(
      const GameEndAnimatorConfig &cfg,
      const RoboCommonLayoutOutInterface &outInterface,
      RoboCommonLayout &layout);

  static ErrorCode initFogOfWar(
      const RoboCommonLayoutConfig &layoutCfg,
      const RoboCommonLayoutOutInterface &outInterface,
      RoboCommonLayout &layout);

  static ErrorCode initPlayerRobot(
      const RoboCommonLayoutConfig &layoutCfg,
      const RoboCommonLayoutOutInterface &outInterface,
      RoboCommonLayout &layout);

  static ErrorCode initDebugField(const RoboCommonLayoutConfig &layoutCfg,
                                  RoboCommonLayout &layout);
};

#endif /* ROBO_COMMON_ROBOCOMMONLAYOUTINITHELPER_H_ */
