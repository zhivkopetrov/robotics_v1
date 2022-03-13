#ifndef ROBO_CLEANER_GUI_ROBOCLEANERGUIINITHELPER_H_
#define ROBO_CLEANER_GUI_ROBOCLEANERGUIINITHELPER_H_

//System headers
#include <cstdint>
#include <any>

//Other libraries headers
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations
class RoboCleanerGui;
struct RoboCleanerLayoutConfig;
struct RoboCleanerLayoutInterface;

class RoboCleanerGuiInitHelper {
public:
  RoboCleanerGuiInitHelper() = delete;

  static ErrorCode init(const std::any &cfg, RoboCleanerGui &gui);

private:
  static ErrorCode initLayout(const RoboCleanerLayoutConfig &cfg,
                              RoboCleanerLayoutInterface &interface, //out param
                              RoboCleanerGui &gui);

  static ErrorCode initControllerExternalBridge(
      const RoboCleanerLayoutInterface &interface, RoboCleanerGui &gui);
};

#endif /* ROBO_CLEANER_GUI_ROBOCLEANERGUIINITHELPER_H_ */
