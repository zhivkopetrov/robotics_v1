//Corresponding header
#include "robo_common/layout/field/obstacles/Obstacle.h"

//System headers

//Other libraries headers
#include "manager_utils/drawing/Fbo.h"
#include "utils/Log.h"

//Own components headers
#include "robo_common/layout/field/FieldUtils.h"
#include "robo_common/helpers/CollisionWatcher.h"

ErrorCode Obstacle::init(const ObstacleConfig &cfg,
                         const FieldDescription &fieldDescr,
                         CollisionWatcher *collisionWatcher) {
  _img.create(cfg.rsrcId);
  _img.activateScaling();
  _img.setScaledWidth(cfg.width);
  _img.setScaledHeight(cfg.height);

  auto pos = FieldUtils::getAbsPos(cfg.fieldPos, fieldDescr);
  pos += cfg.tileOffset;
  _img.setPosition(pos);

  _collisionObjHandle = collisionWatcher->registerObject(this,
      CollisionDamageImpact::YES);

  return ErrorCode::SUCCESS;
}

void Obstacle::drawOnFbo(Fbo &fbo) const {
  fbo.addWidget(_img);
}

Rectangle Obstacle::getBoundary() const {
  return _img.getScaledRect();
}

void Obstacle::registerCollision([[maybe_unused]]const Rectangle &intersectRect,
                                 [[maybe_unused]]CollisionDamageImpact impact) {
  //Obstacle class is just serving as a ... well ... an obstacle
}
