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
    const ObstacleOutInterface& interface) {
  createImage(cfg, fieldDescr);

  if (nullptr == interface.collisionWatcher) {
    LOGERR("Error, nullptr provided for collisionWatcher");
    return ErrorCode::FAILURE;
  }
  _collisionObjHandle = interface.collisionWatcher->registerObject(this,
      CollisionDamageImpact::YES);

  _overlayStatus = cfg.status;
  _obstacleVisibility = cfg.obstacleVisibility;

  if (ObstacleHandlerApproachOverlayStatus::ENABLED == _overlayStatus) {
    if (ErrorCode::SUCCESS != createObjOverlay(cfg, interface, fieldDescr)) {
      LOGERR("Error, _objApproachOverlay.init() failed");
      return ErrorCode::FAILURE;
    }
  }

  return ErrorCode::SUCCESS;
}

void Obstacle::draw() const {
  if (ObstacleVisibility::DEFAULT == _obstacleVisibility) {
    _img.draw();
  }

#if DEBUG_VISUAL_OVERLAY
  if (ObstacleHandlerApproachOverlayStatus::ENABLED == _overlayStatus) {
    _objApproachOverlay.draw();
  }
#endif //DEBUG_VISUAL_OVERLAY
}

void Obstacle::drawOnFbo(Fbo &fbo) const {
  if (ObstacleVisibility::DEFAULT == _obstacleVisibility) {
    fbo.addWidget(_img);
  }

#if DEBUG_VISUAL_OVERLAY
  if (ObstacleHandlerApproachOverlayStatus::ENABLED == _overlayStatus) {
    _objApproachOverlay.drawOnFbo(fbo);
  }
#endif //DEBUG_VISUAL_OVERLAY
}

Rectangle Obstacle::getBoundary() const {
  return _img.getScaledRect();
}

void Obstacle::registerCollision([[maybe_unused]]const Rectangle &intersectRect,
                                 [[maybe_unused]]CollisionDamageImpact impact) {
  //Obstacle class is just serving as a ... well ... an obstacle
}

void Obstacle::createImage(const ObstacleConfig &cfg,
                           const FieldDescription &fieldDescr) {
  _img.create(cfg.rsrcId);
  _img.activateScaling();
  _img.setScaledWidth(cfg.width);
  _img.setScaledHeight(cfg.height);

  const auto absPos =
      FieldUtils::getAbsPos(cfg.fieldPos, fieldDescr) + cfg.tileOffset;
  _img.setPosition(absPos);
}

ErrorCode Obstacle::createObjOverlay(const ObstacleConfig &cfg,
                                     const ObstacleOutInterface &interface,
                                     const FieldDescription &fieldDescr) {
  const auto absFieldPos = FieldUtils::getAbsPos(cfg.fieldPos, fieldDescr);

  const ObjectApproachOverlayConfig objOverlayCfg = {
    .preScaledOverlayBoundary = _img.getScaledRect(),
    .upperBoundary = Rectangle(absFieldPos, cfg.tileWidth, cfg.tileHeight),
    .scaleFactor = cfg.objApproachOverlayScaleFactor,
    .fieldPos = cfg.fieldPos
  };

  const ObjectApproachOverlayOutInterface objOverLayOutInterface = {
    .objectApproachOverlayTriggeredCb =
        interface.objectApproachOverlayTriggeredCb,
    .containerRedrawCb = interface.containerRedrawCb,
    .collisionWatcher = interface.collisionWatcher
  };

  if (ErrorCode::SUCCESS != _objApproachOverlay.init(objOverlayCfg,
      objOverLayOutInterface)) {
    LOGERR("Error, _objApproachOverlay.init() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}
