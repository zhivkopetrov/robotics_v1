#ifndef ROBO_CLEANER_GUI_ROBOCLEANERGUICONFIGGENERATOR_H_
#define ROBO_CLEANER_GUI_ROBOCLEANERGUICONFIGGENERATOR_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "game_engine/config/ApplicationConfig.h"

//Own components headers

//Forward declarations

class RoboCleanerGuiConfigGenerator {
public:
  RoboCleanerGuiConfigGenerator() = delete;

  static std::vector<DependencyDescription> generateDependencies(int32_t argc,
                                                                 char **args);

  static ApplicationConfig generateConfig();
};

#endif /* ROBO_CLEANER_GUI_ROBOCLEANERGUICONFIGGENERATOR_H_ */
