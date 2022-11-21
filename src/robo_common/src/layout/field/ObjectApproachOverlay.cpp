//Corresponding header
#include "robo_common/layout/field/ObjectApproachOverlay.h"

//System headers

//Other libraries headers
#include "utils/drawing/WidgetAligner.h"
#include "utils/debug/StackTrace.h"
#include "utils/Log.h"

//Own components headers
#include "robo_common/helpers/CollisionWatcher.h"

ErrorCode ObjectApproachOverlay::init(
    const ObjectApproachOverlayConfig &cfg,
    const ObjectApproachOverlayOutInterface &interface) {
  _cfg = cfg;

  if (ErrorCode::SUCCESS != initOutInterface(interface)) {
    LOGERR("Error, initOutInterface() failed");
    return ErrorCode::FAILURE;
  }

  _collisionObjHandle = _outInterface.collisionWatcher->registerObject(this,
      CollisionDamageImpact::NO);

  if (ErrorCode::SUCCESS != alignWidget()) {
    LOGERR("Error, alignWidget() failed");
    return ErrorCode::FAILURE;
  }

#if DEBUG_VISUAL_OVERLAY
  _visualFbo.create(_boundary);
  _visualFbo.activateAlphaModulation();
  _visualFbo.setOpacity(FULL_OPACITY / 2);
  _visualFbo.setResetColor(Colors::GREEN);
  _visualFbo.unlock();
  _visualFbo.reset();
  _visualFbo.lock();
#endif //DEBUG_VISUAL_OVERLAY

  return ErrorCode::SUCCESS;
}

void ObjectApproachOverlay::changeBoundary(const Rectangle &preScaledBoundary) {
  _cfg.preScaledOverlayBoundary = preScaledBoundary;
  if (ErrorCode::SUCCESS != alignWidget()) {
    LOGERR("Error, alignWidget() failed. Printing Stacktrace");
    printStacktrace();
  }
}

#if DEBUG_VISUAL_OVERLAY
void ObjectApproachOverlay::draw() const {
  _visualFbo.draw();
}

void ObjectApproachOverlay::drawOnFbo(Fbo &fbo) const {
  fbo.addWidget(_visualFbo);
}
#endif //DEBUG_VISUAL_OVERLAY

ErrorCode ObjectApproachOverlay::initOutInterface(
    const ObjectApproachOverlayOutInterface &interface) {
  _outInterface = interface;
  if (nullptr == _outInterface.objectApproachOverlayTriggeredCb) {
    LOGERR("Error, nullptr provided for ObjechApproachOverlayTriggeredCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.containerRedrawCb) {
    LOGERR("Error, nullptr provided for ContainerRedrawCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.collisionWatcher) {
    LOGERR("Error, nullptr provided for CollisionWatcher");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

void ObjectApproachOverlay::registerCollision(
    [[maybe_unused]]const Rectangle &intersectRect,
    [[maybe_unused]]CollisionDamageImpact impact) {
  deactivate();
  _outInterface.objectApproachOverlayTriggeredCb(_cfg.fieldPos);
}

Rectangle ObjectApproachOverlay::getBoundary() const {
  return _boundary;
}

void ObjectApproachOverlay::deactivate() {
  _outInterface.collisionWatcher->unregisterObject(_collisionObjHandle);
  _collisionObjHandle = INVALID_COLLISION_OBJ_HANDLE;

#if DEBUG_VISUAL_OVERLAY
  _visualFbo.hide();
  _outInterface.containerRedrawCb();
#endif //DEBUG_VISUAL_OVERLAY
}

ErrorCode ObjectApproachOverlay::alignWidget() {
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
    return ErrorCode::FAILURE;
  }

  if ((0 == _boundary.w) || (0 == _boundary.w)) {
    LOGERR("Error, ObjectApproachOverlay with zero width/height detected "
        "[width: %d, height: %d]", _boundary.w, _boundary.h);
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}
