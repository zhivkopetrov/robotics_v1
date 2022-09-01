//System headers
#include <cstdint>

//Other libraries headers
#include "utils/Log.h"

//Own components headers
#include "urscript_bridge/core/UrBridgeApplication.h"
#include "urscript_bridge/config/UrBridgeConfigGenerator.h"

int32_t main(int32_t argc, char **args) {
  UrBridgeApplication app;

  const auto dependencies = UrBridgeConfigGenerator::generateDependencies(
      argc, args);
  if (ErrorCode::SUCCESS != app.loadDependencies(dependencies)) {
    LOGERR("app.loadDependencies() failed");
    return EXIT_FAILURE;
  }

  const auto cfg = UrBridgeConfigGenerator::generateConfig();
  if (ErrorCode::SUCCESS != app.init(cfg)) {
    LOGERR("app.init() failed");
    return EXIT_FAILURE;
  }

  if (ErrorCode::SUCCESS != app.run()) {
    LOGERR("app.run() failed");
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
