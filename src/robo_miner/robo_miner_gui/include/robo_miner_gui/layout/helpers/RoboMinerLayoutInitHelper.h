#ifndef ROBO_MINER_GUI_ROBOMINERLAYOUTINITHELPER_H_
#define ROBO_MINER_GUI_ROBOMINERLAYOUTINITHELPER_H_

//System headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "utils/ErrorCode.h"

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

  static ErrorCode init(const RoboMinerLayoutConfig &cfg,
                        const RoboMinerLayoutOutInterface &outInterface,
                        RoboCommonLayoutInterface &commonInterface, //out param
                        RoboMinerLayout &layout);

private:
  static ErrorCode initPanelHandler(
      const PanelHandlerConfig &cfg,
      const RoboMinerLayoutOutInterface &outInterface,
      const RoboCommonLayoutInterface &commonInterface,
      RoboMinerLayout &layout);

  static ErrorCode initCrystalHandler(
      uint64_t crystalRsrcId, const RoboCommonLayoutInterface &commonInterface,
      RoboMinerLayout &layout);
};

#endif /* ROBO_MINER_GUI_ROBOMINERLAYOUTINITHELPER_H_ */
