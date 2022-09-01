#ifndef URSCRIPT_BRIDGE_URSCRIPTCONFIGGENERATOR_H_
#define URSCRIPT_BRIDGE_URSCRIPTCONFIGGENERATOR_H_

//System headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "game_engine/defines/DependencyDefines.h"

//Own components headers
#include "urscript_bridge/config/UrBridgeConfig.h"

//Forward declarations

class UrBridgeConfigGenerator {
public:
  UrBridgeConfigGenerator() = delete;

  static std::vector<DependencyDescription> generateDependencies(int32_t argc,
                                                                 char **args);

  static UrBridgeConfig generateConfig();
};

#endif /* URSCRIPT_BRIDGE_URSCRIPTCONFIGGENERATOR_H_ */
