#ifndef ROBO_CLEANER_GUI_ROBOCLEANERLAYOUTINITHELPER_H_
#define ROBO_CLEANER_GUI_ROBOCLEANERLAYOUTINITHELPER_H_

//System headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations
class RoboCleanerLayout;
struct RoboCleanerLayoutOutInterface;
struct RoboCleanerLayoutConfig;
struct PanelHandlerConfig;
struct RoboCommonLayoutInterface;

class RoboCleanerLayoutInitHelper {
public:
  RoboCleanerLayoutInitHelper() = delete;

  static ErrorCode init(const RoboCleanerLayoutConfig &cfg,
                        const RoboCleanerLayoutOutInterface &outInterface,
                        RoboCommonLayoutInterface &commonInterface, //out param
      RoboCleanerLayout &layout);

private:
  static ErrorCode initEntityHandler(
      const RoboCleanerLayoutConfig &layoutCfg,
      const RoboCleanerLayoutOutInterface &outInterface,
      RoboCleanerLayout &layout);

  static ErrorCode initPanelHandler(
      const PanelHandlerConfig &cfg,
      const RoboCommonLayoutInterface &commonInterface,
      const RoboCleanerLayoutOutInterface &outInterface,
      RoboCleanerLayout &layout);
};

#endif /* ROBO_CLEANER_GUI_ROBOCLEANERLAYOUTINITHELPER_H_ */
