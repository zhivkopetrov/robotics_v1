//Corresponding header
#include "robo_common/layout/field/obstacles/Obstacle.h"

//System headers

//Other libraries headers
#include "manager_utils/drawing/Fbo.h"
#include "utils/Log.h"

//Own components headers
#include "robo_common/layout/field/FieldUtils.h"
#include "robo_common/helpers/CollisionWatcher.h"

ErrorCode Obstacle::init(
    const ObstacleConfig &cfg, const FieldDescription &fieldDescr,
    CollisionWatcher *collisionWatcher,
    const ObjechApproachOverlayTriggeredCb &objechApproachOverlayTriggeredCb) {
  _img.create(cfg.rsrcId);
  _img.activateScaling();
  _img.setScaledWidth(cfg.width);
  _img.setScaledHeight(cfg.height);

  const auto absFieldPos = FieldUtils::getAbsPos(cfg.fieldPos, fieldDescr);
  const auto obstaclePos =
      Point(absFieldPos.x + cfg.tileOffset.x, absFieldPos.y + cfg.tileOffset.y);
  _img.setPosition(obstaclePos);

  _collisionObjHandle = collisionWatcher->registerObject(this,
      CollisionDamageImpact::YES);

  if (ObstacleHandlerApproachOverlayStatus::ENABLED == cfg.status) {
    LOGM("ObstacleHandlerApproachOverlayStatus::ENABLED");

    const ObjectApproachOverlayConfig objOverlayCfg = {
      .preScaledOverlayBoundary = _img.getScaledRect(),
      .upperBoundary = Rectangle(absFieldPos, cfg.tileWidth, cfg.tileHeight),
      .scaleFactor = cfg.objApproachOverlayScaleFactor,
      .fieldPos = cfg.fieldPos,
      .isInsideField = true,
      .collisionWatcher = collisionWatcher
    };

    if (ErrorCode::SUCCESS != _objApproachOverlay.init(objOverlayCfg,
            objechApproachOverlayTriggeredCb)) {
      LOGERR("Error, _objApproachOverlay.init() failed");
      return ErrorCode::FAILURE;
    }
  }

  return ErrorCode::SUCCESS;
}

void Obstacle::drawOnFbo(Fbo &fbo) const {
  fbo.addWidget(_img);

#if DEBUG_VISUAL_OVERLAY
  _objApproachOverlay.drawOnFbo(fbo);
#endif //DEBUG_VISUAL_OVERLAY
}

Rectangle Obstacle::getBoundary() const {
  return _img.getScaledRect();
}

void Obstacle::registerCollision([[maybe_unused]]const Rectangle &intersectRect,
                                 [[maybe_unused]]CollisionDamageImpact impact) {
  //Obstacle class is just serving as a ... well ... an obstacle
}
