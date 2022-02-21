#ifndef ROBO_COLLECTOR_COMMON_ROBOCOLLECTORCONTROLLERBASECONFIG_H_
#define ROBO_COLLECTOR_COMMON_ROBOCOLLECTORCONTROLLERBASECONFIG_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>

//Other libraries headers

//Own components headers

//Forward declarations

enum class LocalControllerMode {
  ENABLED,
  DISABLED
};

struct RoboCollectorUiControllerBaseConfig {
  std::vector<uint64_t> moveButtonsRsrcIds;
  uint64_t moveButtonInfoTextFontId = 0;

  uint64_t horDelimiterRsrcId = 0;
  uint64_t vertDelimiterRsrcId = 0;

  uint64_t helpButtonRsrcId = 0;
  uint64_t settingsButtonRsrcId = 0;

  LocalControllerMode localControllerMode = LocalControllerMode::DISABLED;
};

#endif /* ROBO_COLLECTOR_COMMON_ROBOCOLLECTORCONTROLLERBASECONFIG_H_ */
