//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "ros2_game_engine/communicator/Ros2Communicator.h"
#include "game_engine/Application.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_controller/helpers/RoboCollectorControllerBuilder.h"
#include "robo_collector_controller/config/RoboCollectorControllerConfigGenerator.h"

int32_t main(int32_t argc, char **args) {
  Application app;

  const auto dependencies =
      RoboCollectorControllerConfigGenerator::generateDependencies(argc, args);
  if (SUCCESS != app.loadDependencies(dependencies)) {
    LOGERR("app.loadDependencies() failed");
    return FAILURE;
  }

  auto communicator = std::make_unique<Ros2Communicator>();
  auto game = RoboCollectorControllerBuilder::createRoboCollectorController(
      communicator);
  app.obtain(std::move(game), std::move(communicator));

  const auto cfg = RoboCollectorControllerConfigGenerator::generateConfig();
  if (SUCCESS != app.init(cfg)) {
    LOGERR("app.init() failed");
    return FAILURE;
  }

  return app.run();
}
