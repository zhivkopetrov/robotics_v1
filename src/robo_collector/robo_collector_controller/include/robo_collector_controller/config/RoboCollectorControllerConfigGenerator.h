#ifndef ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERCONFIGGENERATOR_H_
#define ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERCONFIGGENERATOR_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "game_engine/config/ApplicationConfig.h"

//Own components headers

//Forward declarations

class RoboCollectorControllerConfigGenerator {
public:
  RoboCollectorControllerConfigGenerator() = delete;

  static std::vector<DependencyDescription> generateDependencies(int32_t argc,
                                                                 char **args);

  static ApplicationConfig generateConfig();
};

#endif /* ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERCONFIGGENERATOR_H_ */
