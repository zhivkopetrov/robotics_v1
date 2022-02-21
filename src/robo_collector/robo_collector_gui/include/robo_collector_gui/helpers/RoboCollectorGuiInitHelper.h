#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUIINITHELPER_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUIINITHELPER_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <any>

//Other libraries headers

//Own components headers

//Forward declarations
class RoboCollectorGui;
struct RoboCollectorLayoutConfig;
struct RoboCollectorLayoutInterface;

class RoboCollectorGuiInitHelper {
public:
  RoboCollectorGuiInitHelper() = delete;

  static int32_t init(const std::any &cfg, RoboCollectorGui &gui);

private:
  static int32_t initLayout(const RoboCollectorLayoutConfig &cfg,
                            RoboCollectorLayoutInterface &interface, //out param
      RoboCollectorGui &gui);
  static int32_t initTurnHelper(const RoboCollectorLayoutInterface &interface,
                                char fieldEnemyMarker,
                                RoboCollectorGui &gui);
  static int32_t initControllerExternalBridge(
      const RoboCollectorLayoutInterface &interface, RoboCollectorGui &gui);
};

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUIINITHELPER_H_ */
