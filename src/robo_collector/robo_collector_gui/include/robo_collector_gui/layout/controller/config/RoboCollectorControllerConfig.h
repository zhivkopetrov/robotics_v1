#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORCONTROLLERCONFIG_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORCONTROLLERCONFIG_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>

//Other libraries headers

//Own components headers

//Forward declarations

struct RoboCollectorControllerConfig {
  std::vector<uint64_t> moveButtonsRsrcIds;
  uint64_t moveButtonInfoTextFontId = 0;
  int32_t maxMoveButtons = 0;

  uint64_t horDelimiterRsrcId = 0;
  uint64_t vertDelimiterRsrcId = 0;

  uint64_t helpButtonRsrcId = 0;
  uint64_t settingsButtonRsrcId = 0;

  bool isEnabled = false;
};

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORCONTROLLERCONFIG_H_ */
