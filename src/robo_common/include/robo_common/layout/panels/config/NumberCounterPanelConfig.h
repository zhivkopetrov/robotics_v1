#ifndef ROBO_COMMON_NUMBERCOUNTERPANELCONFIG_H_
#define ROBO_COMMON_NUMBERCOUNTERPANELCONFIG_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers

struct NumberCounterPanelConfig {
  uint64_t targetNumber = 0;
  uint64_t rsrcId = 0;
  uint64_t fontId = 0;
  int32_t incrTimerId = 0;
  int32_t decrTimerId = 0;
};

#endif /* ROBO_COMMON_NUMBERCOUNTERPANELCONFIG_H_ */
