#ifndef ROBO_COMMON_INDICATORPANELCONFIG_H_
#define ROBO_COMMON_INDICATORPANELCONFIG_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers

struct IndicatorPanelConfig {
  uint64_t rsrcId = 0;
  uint64_t indicatorRsrcId = 0;
  uint64_t indicatorFontId = 0;
  int32_t indicatorReduceTimerId = 0;
};

#endif /* ROBO_COMMON_INDICATORPANELCONFIG_H_ */
