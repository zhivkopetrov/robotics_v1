#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUICONFIG_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUICONFIG_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>

//Other libraries headers

//Own components headers
#include "robo_collector_gui/field/config/FieldConfig.h"
#include "robo_collector_gui/panels/config/PanelConfig.h"

//Forward declarations

struct RoboCollectorGuiConfig {
  FieldConfig fieldCfg;
  PanelConfig panelConfig;
  uint64_t mapRsrcId = 0;
  uint64_t robotBlinkyRsrcId = 0;
  uint64_t robotEnemiesRsrcId = 0;
  uint64_t robotsAnimStartTimerId = 0;

  std::vector<uint64_t> moveButtonsRsrcIds;
  int32_t maxMoveButtons = 0;

  std::vector<uint64_t> coinAnimRsrcIds;
  int32_t maxCoins = 0;
  int32_t coinRotateAnimFirstTimerId = 0;
  int32_t coinCollectAnimFirstTimerId = 0;

  char blinkyFieldMarker = 'B';
  char enemyFieldMarker = 'E';
};

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUICONFIG_H_ */

