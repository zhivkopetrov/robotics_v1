#ifndef ROBO_MINER_GUI_ROBOMINERGUICONFIGGENERATOR_H_
#define ROBO_MINER_GUI_ROBOMINERGUICONFIGGENERATOR_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "game_engine/config/ApplicationConfig.h"

//Own components headers

//Forward declarations

class RoboMinerGuiConfigGenerator {
public:
  RoboMinerGuiConfigGenerator() = delete;

  static std::vector<DependencyDescription> generateDependencies(int32_t argc,
                                                                 char **args);

  static ApplicationConfig generateConfig();
};

#endif /* ROBO_MINER_GUI_ROBOMINERGUICONFIGGENERATOR_H_ */
