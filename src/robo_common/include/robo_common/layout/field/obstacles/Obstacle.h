#ifndef ROBO_COMMON_OBSTACLE_H_
#define ROBO_COMMON_OBSTACLE_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "manager_utils/drawing/Image.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "robo_common/defines/RoboCommonDefines.h"
#include "robo_common/layout/field/FieldPos.h"
#include "robo_common/helpers/CollisionObject.h"

//Forward declarations
class CollisionWatcher;
class Fbo;

struct ObstacleConfig {
  uint64_t rsrcId { };
  FieldPos fieldPos;
  Point tileOffset;
  int32_t width { };
  int32_t height { };
};

class Obstacle final : public CollisionObject {
public:
  ErrorCode init(const ObstacleConfig &cfg, const FieldDescription &fieldDescr,
                 CollisionWatcher *collisionWatcher);

  void drawOnFbo(Fbo &fbo) const;

  Rectangle getBoundary() const override;

private:
  void registerCollision(const Rectangle &intersectRect,
                         CollisionDamageImpact impact) override;

  Image _img;
};

#endif /* ROBO_COMMON_OBSTACLE_H_ */
