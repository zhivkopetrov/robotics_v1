//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "ros2_game_engine/Ros2Application.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/RoboCollectorGui.h"
#include "robo_collector_gui/config/RoboCollectorGuiConfigGenerator.h"

int32_t main(int32_t argc, char **args) {
  std::unique_ptr<Game> game = std::make_unique<RoboCollectorGui>();
  Ros2Application app(std::move(game));

  const auto dependencies =
      RoboCollectorGuiConfigGenerator::generateDependencies(argc, args);
  if (SUCCESS != app.loadDependencies(dependencies)) {
    LOGERR("app.loadDependencies() failed");
    return FAILURE;
  }

  const auto cfg = RoboCollectorGuiConfigGenerator::generateConfig();
  if (SUCCESS != app.init(cfg)) {
    LOGERR("Ros2Application.init() failed");
    return FAILURE;
  }

  return app.run();
}
