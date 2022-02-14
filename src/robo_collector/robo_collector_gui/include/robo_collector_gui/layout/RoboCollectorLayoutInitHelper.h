#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUTINITHELPER_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUTINITHELPER_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>

//Other libraries headers

//Own components headers

//Forward declarations
class RoboCollectorLayout;
struct RoboCollectorLayoutOutInterface;
struct RoboCollectorLayoutConfig;
struct PanelHandlerConfig;
struct CoinHandlerConfig;
struct RoboCollectorControllerConfig;

class RoboCollectorLayoutInitHelper {
public:
  RoboCollectorLayoutInitHelper() = delete;

  static int32_t init(const RoboCollectorLayoutConfig &cfg,
                      const RoboCollectorLayoutOutInterface &interface,
                      RoboCollectorLayout &layout);

private:
  static int32_t initRobots(const RoboCollectorLayoutConfig &layoutCfg,
                            const RoboCollectorLayoutOutInterface &interface,
                            RoboCollectorLayout &layout);

  static int32_t initPanelHandler(const PanelHandlerConfig &cfg,
                                  RoboCollectorLayout &layout);

  static int32_t initCoinHandler(
      const CoinHandlerConfig &cfg,
      const RoboCollectorLayoutOutInterface &interface,
      RoboCollectorLayout &layout);

  static int32_t initController(const RoboCollectorControllerConfig &guiCfg,
                                RoboCollectorLayout &layout);
};

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUTINITHELPER_H_ */
