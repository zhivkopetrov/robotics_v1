//Corresponding header
#include "robo_common/layout/field/fog_of_war/FogCollisionObject.h"

//System headers

//Other libraries headers
#include "manager_utils/drawing/Image.h"
#include "utils/Log.h"

//Own components headers
#include "robo_common/helpers/CollisionWatcher.h"

namespace {
constexpr auto OPACITY_STEP = 15;
}

ErrorCode FogCollisionObject::init(
    const Rectangle &boundary, int32_t id, int32_t timerId,
    const OnFogObjectAimCompleteCb &onAnimCompleteCb,
    CollisionWatcher *collisionWatcher) {
  if (nullptr == collisionWatcher) {
    LOGERR("Error, nullptr provided for collisionWatcher");
    return ErrorCode::FAILURE;
  }
  _collisionWatcher = collisionWatcher;
  _collisionObjHandle =
      _collisionWatcher->registerObject(this, CollisionDamageImpact::NO);

  if (nullptr == onAnimCompleteCb) {
    LOGERR("Error, nullptr provided for OnFogObjectAimCompleteCb");
    return ErrorCode::FAILURE;
  }
  _onAnimCompleteCb = onAnimCompleteCb;

  _id = id;
  _timerId = timerId;

  Fbo::create(boundary);
  Fbo::activateAlphaModulation();
  Fbo::setResetColor(Colors::FULL_TRANSPARENT);

  return ErrorCode::SUCCESS;
}

void FogCollisionObject::populateVisualContent(const Image &fogImg) {
  Fbo::unlock();
  Fbo::reset(); //because of the reset color (FULL_TRANSPARENT)

  Fbo::addWidget(fogImg);

  Fbo::update();
  Fbo::lock();
}

Rectangle FogCollisionObject::getBoundary() const {
  return Fbo::getImageRect();
}

void FogCollisionObject::registerCollision(
    [[maybe_unused]]const Rectangle &intersectRect,
    [[maybe_unused]]CollisionDamageImpact impact) {
  _collisionWatcher->unregisterObject(_collisionObjHandle);
  _collisionObjHandle = INVALID_COLLISION_OBJ_HANDLE;

  //start fade out animation
  startTimer(50, _timerId, TimerType::PULSE);
}

void FogCollisionObject::onTimeout(const int32_t timerId) {
  if (_timerId != timerId) {
    LOGERR("Error, received unsupported timerId: %d", timerId);
    return;
  }

  const int32_t opacity = getOpacity() - OPACITY_STEP;
  if (0 == opacity) {
    stopTimer(_timerId);
    _onAnimCompleteCb(_id);
    return;
  }

  setOpacity(opacity);
}

