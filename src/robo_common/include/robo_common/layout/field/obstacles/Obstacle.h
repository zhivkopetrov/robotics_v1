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
#include "robo_common/layout/field/ObjectApproachOverlay.h"
#include "robo_common/helpers/CollisionObject.h"

//Forward declarations
class CollisionWatcher;
class Fbo;

enum class ObstacleVisibility {
  DEFAULT, HIDDEN
};

struct ObstacleConfig {
  uint64_t rsrcId { };
  FieldPos fieldPos;
  Point tileOffset;
  double objApproachOverlayScaleFactor { };
  int32_t width { };
  int32_t height { };
  int32_t tileWidth { };
  int32_t tileHeight { };
  ObstacleHandlerApproachOverlayStatus status =
      ObstacleHandlerApproachOverlayStatus::DISABLED;
  ObstacleVisibility obstacleVisibility = ObstacleVisibility::DEFAULT;
};

class Obstacle final : public CollisionObject {
public:
  ErrorCode init(
      const ObstacleConfig &cfg, const FieldDescription &fieldDescr,
      CollisionWatcher *collisionWatcher,
      const ObjechApproachOverlayTriggeredCb &objechApproachOverlayTriggeredCb);

  void drawOnFbo(Fbo &fbo) const;

private:
  Rectangle getBoundary() const override;

  void registerCollision(const Rectangle &intersectRect,
                         CollisionDamageImpact impact) override;

  Image _img;
  ObjectApproachOverlay _objApproachOverlay;
  ObstacleVisibility _obstacleVisibility = ObstacleVisibility::DEFAULT;
  ObstacleHandlerApproachOverlayStatus _overlayStatus =
      ObstacleHandlerApproachOverlayStatus::DISABLED;
};

#endif /* ROBO_COMMON_OBSTACLE_H_ */
