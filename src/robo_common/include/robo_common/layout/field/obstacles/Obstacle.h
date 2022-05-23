#ifndef ROBO_COMMON_OBSTACLE_H_
#define ROBO_COMMON_OBSTACLE_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "robo_common/defines/RoboCommonDefines.h"
#include "robo_common/layout/field/FieldPos.h"
#include "manager_utils/drawing/Image.h"
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations
class Fbo;

struct ObstacleConfig {
  uint64_t rsrcId { };
  FieldPos fieldPos;
  Point tileOffset;
  int32_t width { };
  int32_t height { };
};

class Obstacle {
public:
  ErrorCode init(const ObstacleConfig &cfg, const FieldDescription &fieldDescr);

  void drawOnFbo(Fbo &fbo) const;

private:
  Image _img;
};

#endif /* ROBO_COMMON_OBSTACLE_H_ */
