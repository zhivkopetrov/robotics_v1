//Corresponding header
#include "robo_common/layout/field/ObjectApproachOverlay.h"

//System headers

//Other libraries headers
#include "utils/drawing/WidgetAligner.h"
#include "utils/Log.h"

//Own components headers
#include "robo_common/helpers/CollisionWatcher.h"

ErrorCode ObjectApproachOverlay::init(
    const ObjectApproachOverlayConfig &cfg,
    const ObjechApproachOverlayTriggeredCb &objechApproachOverlayTriggeredCb) {
  if (nullptr == objechApproachOverlayTriggeredCb) {
    LOGERR("Error, nullptr provided for ObjechApproachOverlayTriggeredCb");
    return ErrorCode::FAILURE;
  }
  _objechApproachOverlayTriggeredCb = objechApproachOverlayTriggeredCb;

  if (nullptr == cfg.collisionWatcher) {
    LOGERR("Error, nullptr provided for CollisionWatcher");
    return ErrorCode::FAILURE;
  }
  _cfg = cfg;
  _collisionObjHandle = _cfg.collisionWatcher->registerObject(this,
      CollisionDamageImpact::NO);

  alignWidget();
#if DEBUG_VISUAL_OVERLAY
  _visualFbo.create(_boundary);
  _visualFbo.activateAlphaModulation();
  _visualFbo.setOpacity(FULL_OPACITY / 2);
  _visualFbo.unlock();
  _visualFbo.setResetColor(Colors::GREEN);
  _visualFbo.reset();
  _visualFbo.lock();
#endif //DEBUG_VISUAL_OVERLAY

  return ErrorCode::SUCCESS;
}

void ObjectApproachOverlay::changeBoundary(const Rectangle &preScaledBoundary) {
  _cfg.preScaledOverlayBoundary = preScaledBoundary;
  alignWidget();
}

#if DEBUG_VISUAL_OVERLAY
void ObjectApproachOverlay::drawOnFbo([[maybe_unused]]Fbo &fbo) const {
  fbo.addWidget(_visualFbo);
}
#endif //DEBUG_VISUAL_OVERLAY

void ObjectApproachOverlay::registerCollision(
    [[maybe_unused]]const Rectangle &intersectRect,
    [[maybe_unused]]CollisionDamageImpact impact) {
  deactivate();
  _objechApproachOverlayTriggeredCb(_cfg.fieldPos);
}

Rectangle ObjectApproachOverlay::getBoundary() const {
  return _boundary;
}

void ObjectApproachOverlay::deactivate() {
  _cfg.collisionWatcher->unregisterObject(_collisionObjHandle);
  _collisionObjHandle = INVALID_COLLISION_OBJ_HANDLE;
}

void ObjectApproachOverlay::alignWidget() {
  _boundary.w =
      static_cast<int32_t>(_cfg.preScaledOverlayBoundary.w * _cfg.scaleFactor);
  _boundary.h =
      static_cast<int32_t>(_cfg.preScaledOverlayBoundary.h * _cfg.scaleFactor);

  const Point boudaryPos = WidgetAligner::getPosition(_boundary.w, _boundary.h,
      _cfg.upperBoundary, WidgetAlignment::CENTER_CENTER);
  _boundary.x = boudaryPos.x;
  _boundary.y = boudaryPos.y;

  if ((_boundary.w >= _cfg.upperBoundary.w) ||
      (_boundary.h >= _cfg.upperBoundary.h)) {
    LOGERR("Error, ObjectApproachOverlay boundary with [width: %d, height: %d] "
           "should be smaller in both dimension from Upper Boundary [width: %d,"
           " height: %d]", _boundary.w, _boundary.h, _cfg.upperBoundary.w,
           _cfg.upperBoundary.h);
  }
}
