#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUIINITHELPER_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUIINITHELPER_H_

//System headers
#include <cstdint>
#include <any>

//Other libraries headers
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations
class RoboCollectorGui;
struct RoboCollectorLayoutConfig;
struct RoboCollectorLayoutInterface;
enum class LocalControllerMode;

class RoboCollectorGuiInitHelper {
public:
  RoboCollectorGuiInitHelper() = delete;

  static ErrorCode init(const std::any &cfg, RoboCollectorGui &gui);

private:
  static ErrorCode initLayout(const RoboCollectorLayoutConfig &cfg,
                              RoboCollectorLayoutInterface &interface, //out param
                              RoboCollectorGui &gui);

  static ErrorCode initTurnHelper(const RoboCollectorLayoutInterface &interface,
                                  LocalControllerMode localControllerMode,
                                  char fieldEnemyMarker, RoboCollectorGui &gui);

  static ErrorCode initControllerExternalBridge(
      const RoboCollectorLayoutInterface &interface, RoboCollectorGui &gui);
};

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUIINITHELPER_H_ */
