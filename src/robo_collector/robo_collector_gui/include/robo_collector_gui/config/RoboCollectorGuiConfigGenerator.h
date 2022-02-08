#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUICONFIGGENERATOR_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUICONFIGGENERATOR_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "game_engine/config/ApplicationConfig.h"

//Own components headers
#include "robo_collector_gui/config/RoboCollectorGuiConfig.h"

//Forward declarations

class RoboCollectorGuiConfigGenerator {
public:
  RoboCollectorGuiConfigGenerator() = delete;

  static std::vector<DependencyDescription> generateDependencies(int32_t argc,
                                                                 char **args);

  static ApplicationConfig generateConfig();
};

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUICONFIGGENERATOR_H_ */
