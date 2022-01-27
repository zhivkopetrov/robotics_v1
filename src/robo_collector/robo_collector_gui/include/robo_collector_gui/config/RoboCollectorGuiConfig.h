#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUICONFIG_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUICONFIG_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>

//Other libraries headers

//Own components headers
#include "robo_collector_gui/field/config/FieldConfig.h"
#include "robo_collector_gui/panels/config/PanelHandlerConfig.h"
#include "robo_collector_gui/entities/robot/config/RobotBaseConfig.h"

//Forward declarations

struct RoboCollectorGuiConfig {
  FieldConfig fieldCfg;
  PanelHandlerConfig panelHandlerCfg;
  RobotBaseConfig robotBaseCfg;

  uint64_t mapRsrcId = 0;

  uint64_t moveButtonsInfoTextFontId = 0;
  std::vector<uint64_t> moveButtonsRsrcIds;
  int32_t maxMoveButtons = 0;

  uint64_t horDelimiterRsrcId = 0;
  uint64_t vertDelimiterRsrcId = 0;

  std::vector<uint64_t> coinAnimRsrcIds;
  std::vector<char> coinFieldDataMarkers;
  int32_t maxCoins = 0;
  int32_t coinRotateAnimFirstTimerId = 0;
  int32_t coinCollectAnimFirstTimerId = 0;
  int32_t coinRespawnAnimFirstTimerId = 0;

  char playerFieldMarker = 'B';
  char enemyFieldMarker = 'E';
};

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUICONFIG_H_ */

