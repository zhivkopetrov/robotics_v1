#ifndef ROBO_MINER_GUI_ROBOMINERLAYOUTINITHELPER_H_
#define ROBO_MINER_GUI_ROBOMINERLAYOUTINITHELPER_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>

//Other libraries headers

//Own components headers

//Forward declarations
class RoboMinerLayout;
struct RoboMinerLayoutOutInterface;
struct RoboMinerLayoutConfig;
struct PanelHandlerConfig;
struct RoboCommonLayoutInterface;

class RoboMinerLayoutInitHelper {
public:
  RoboMinerLayoutInitHelper() = delete;

  static int32_t init(const RoboMinerLayoutConfig &cfg,
                      const RoboMinerLayoutOutInterface &outInterface,
                      RoboCommonLayoutInterface &commonInterface, //out param
      RoboMinerLayout &layout);

private:
  static int32_t initPanelHandler(const PanelHandlerConfig &cfg,
                                  RoboCommonLayoutInterface &commonInterface,
                                  RoboMinerLayout &layout);

  static int32_t initCrystalHandler(uint64_t crystalRsrcId,
                                    RoboCommonLayoutInterface &commonInterface,
                                    RoboMinerLayout &layout);
};

#endif /* ROBO_MINER_GUI_ROBOMINERLAYOUTINITHELPER_H_ */
