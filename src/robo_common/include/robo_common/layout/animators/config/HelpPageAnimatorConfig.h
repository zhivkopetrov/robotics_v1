#ifndef ROBO_COMMON_HELPPAGEANIMATORCONFIG_H_
#define ROBO_COMMON_HELPPAGEANIMATORCONFIG_H_

//System headers
#include <cstdint>
#include <string>
#include <vector>

//Other libraries headers
#include "utils/drawing/Rectangle.h"
#include "utils/drawing/Color.h"

//Own components headers

//Forward declarations

struct HelpPageEntry {
  std::string content;
  Color color = Colors::BLACK;
  uint64_t fontRsrcId { };
  int32_t prependedVerticalSpacing { };
};

struct HelpPageAnimatorConfig {
  uint64_t bgrRsrcId { };
  double bgrToScreenRatio = 0.8;
  Rectangle screenDimensions;
  HelpPageEntry titleEntry;
  std::vector<HelpPageEntry> entries;

  int32_t moveAndFadeAnimTimerId { };
};

#endif /* ROBO_COMMON_HELPPAGEANIMATORCONFIG_H_ */
