#ifndef ROBO_CLEANER_GUI_ROBOCLEANERLAYOUTINITHELPER_H_
#define ROBO_CLEANER_GUI_ROBOCLEANERLAYOUTINITHELPER_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>

//Other libraries headers

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

  static int32_t init(const RoboCleanerLayoutConfig &cfg,
                      const RoboCleanerLayoutOutInterface &outInterface,
                      RoboCommonLayoutInterface &commonInterface, //out param
                      RoboCleanerLayout &layout);

private:
  static int32_t initPanelHandler(const PanelHandlerConfig &cfg,
                                  RoboCommonLayoutInterface &commonInterface,
                                  RoboCleanerLayout &layout);
};

#endif /* ROBO_CLEANER_GUI_ROBOCLEANERLAYOUTINITHELPER_H_ */
