#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUTINITHELPER_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUTINITHELPER_H_

//System headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations
class RoboCollectorLayout;
struct RoboCollectorLayoutOutInterface;
struct RoboCollectorLayoutConfig;
struct PanelHandlerConfig;
struct CoinHandlerConfig;
struct RoboCollectorUiControllerBaseConfig;
struct RoboCommonLayoutInterface;

class RoboCollectorLayoutInitHelper {
public:
  RoboCollectorLayoutInitHelper() = delete;

  static ErrorCode init(const RoboCollectorLayoutConfig &cfg,
                        const RoboCollectorLayoutOutInterface &outInterface,
                        RoboCommonLayoutInterface &commonInterface, //out param
                        RoboCollectorLayout &layout);

private:
  static ErrorCode initRobots(
      const RoboCollectorLayoutConfig &layoutCfg,
      const RoboCollectorLayoutOutInterface &outInterface,
      const RoboCommonLayoutInterface &commonInterface,
      RoboCollectorLayout &layout);

  static ErrorCode initPanelHandler(
      const PanelHandlerConfig &cfg,
      const RoboCollectorLayoutOutInterface &outInterface,
      const RoboCommonLayoutInterface &commonInterface,
      RoboCollectorLayout &layout);

  static ErrorCode initCoinHandler(
      const CoinHandlerConfig &cfg,
      const RoboCollectorLayoutOutInterface &interface,
      const RoboCommonLayoutInterface &commonInterface,
      RoboCollectorLayout &layout);

  static ErrorCode initController(
      const RoboCollectorUiControllerBaseConfig &baseCfg,
      const RoboCommonLayoutInterface &commonInterface,
      RoboCollectorLayout &layout);
};

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUTINITHELPER_H_ */
