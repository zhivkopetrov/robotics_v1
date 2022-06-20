#ifndef ROBO_COMMON_ACHIEVEMENTANIMATORCONFIG_H_
#define ROBO_COMMON_ACHIEVEMENTANIMATORCONFIG_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "utils/drawing/Rectangle.h"

//Own components headers

//Forward declarations

struct AchievementAnimatorConfig {
  Rectangle screenDimensions;
  uint64_t allStarsRsrcId{};
  uint64_t singleStarRsrcId{};
  int32_t fadeAndMoveTimerId{};
};

#endif /* ROBO_COMMON_ACHIEVEMENTANIMATORCONFIG_H_ */
