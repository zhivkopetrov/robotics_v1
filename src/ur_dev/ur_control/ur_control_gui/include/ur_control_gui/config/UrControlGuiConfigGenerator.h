#ifndef UR_CONTROL_GUI_URCONTROLGUICONFIGGENERATOR_H_
#define UR_CONTROL_GUI_URCONTROLGUICONFIGGENERATOR_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "game_engine/config/ApplicationConfig.h"

//Own components headers

//Forward declarations

class UrControlGuiConfigGenerator {
public:
  UrControlGuiConfigGenerator() = delete;

  static std::vector<DependencyDescription> generateDependencies(int32_t argc,
                                                                 char **args);

  static ApplicationConfig generateConfig();
};

#endif /* UR_CONTROL_GUI_URCONTROLGUICONFIGGENERATOR_H_ */
