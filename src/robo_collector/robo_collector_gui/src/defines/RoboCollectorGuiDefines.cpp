//Corresponding header
#include "robo_collector_gui/defines/RoboCollectorGuiDefines.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/data_type/EnumClassUtils.h"
#include "utils/Log.h"

//Own components headers

GameMode toGameMode(const int32_t mode) {
  const GameMode gameMode = toEnum<GameMode>(mode);

  switch (gameMode) {
  case GameMode::NORMAL:
  case GameMode::EXTENDED:
  [[fallthrough]];
  case GameMode::SUPER_EXTENDED:
    return gameMode;
  default:
    LOGERR("Received unknown GameMode code: %d", mode);
    return GameMode::UNKNOWN;
  }
}

