#ifndef ROBO_COMMON_GAMEENDANIMATORCONFIG_H_
#define ROBO_COMMON_GAMEENDANIMATORCONFIG_H_

//System headers
#include <cstdint>
#include <string>

//Other libraries headers
#include "utils/drawing/Rectangle.h"

//Own components headers

//Forward declarations

struct GameEndAnimatorConfig {
  std::string projectName;
  uint64_t bgrRsrcId { };
  uint64_t winStatusFontId { };
  uint64_t userDataFontId { };
  uint64_t countdownFontId { };
  Rectangle screenDimensions;
  int32_t fadeAnimTimerId { };
  int32_t expandAnimTimerId { };
  int32_t countdownAnimTimerId { };
  int32_t levelId { };
  double endScreenToBgrRatio = 0.5;
};

#endif /* ROBO_COMMON_GAMEENDANIMATORCONFIG_H_ */
