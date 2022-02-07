//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "ros2_game_engine/communicator/Ros2Communicator.h"
#include "game_engine/Application.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/builder/RoboCollectorBuilder.h"
#include "robo_collector_gui/config/RoboCollectorGuiConfigGenerator.h"

int32_t main(int32_t argc, char **args) {
  Application app;

  const auto dependencies =
      RoboCollectorGuiConfigGenerator::generateDependencies(argc, args);
  if (SUCCESS != app.loadDependencies(dependencies)) {
    LOGERR("app.loadDependencies() failed");
    return FAILURE;
  }

  auto communicator = std::make_unique<Ros2Communicator>();
  auto game = RoboCollectorBuilder::createRoboCollectorGui(communicator);
  app.obtain(std::move(communicator), std::move(game));

  const auto cfg = RoboCollectorGuiConfigGenerator::generateConfig();
  if (SUCCESS != app.init(cfg)) {
    LOGERR("Ros2Application.init() failed");
    return FAILURE;
  }

  return app.run();
}
