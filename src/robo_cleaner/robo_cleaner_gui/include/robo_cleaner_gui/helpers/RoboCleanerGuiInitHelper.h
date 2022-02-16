#ifndef ROBO_CLEANER_GUI_ROBOCLEANERGUIINITHELPER_H_
#define ROBO_CLEANER_GUI_ROBOCLEANERGUIINITHELPER_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <any>

//Other libraries headers

//Own components headers

//Forward declarations
class RoboCleanerGui;
struct RoboCleanerLayoutConfig;
struct RoboCleanerLayoutInterface;

class RoboCleanerGuiInitHelper {
public:
  RoboCleanerGuiInitHelper() = delete;

  static int32_t init(const std::any &cfg, RoboCleanerGui &gui);

private:
  static int32_t initLayout(const RoboCleanerLayoutConfig &cfg,
                            RoboCleanerLayoutInterface &interface, //out param
                            RoboCleanerGui &gui);

  static int32_t initControllerExternalBridge(
      const RoboCleanerLayoutInterface &interface, RoboCleanerGui &gui);
};

#endif /* ROBO_CLEANER_GUI_ROBOCLEANERGUIINITHELPER_H_ */
