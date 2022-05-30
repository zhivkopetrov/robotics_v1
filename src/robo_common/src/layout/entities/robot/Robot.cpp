//Corresponding header
#include "robo_common/layout/entities/robot/Robot.h"

//System headers

//Other libraries headers
#include "robo_common/layout/field/FieldUtils.h"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_common/layout/entities/robot/helpers/RobotUtils.h"
#include "robo_common/layout/entities/robot/helpers/RobotInitHelper.h"
#include "robo_common/helpers/CollisionWatcher.h"

ErrorCode Robot::init(const RobotState &initialState,
                      const RobotConfig &robotCfg,
                      const RobotAnimatorConfigBase &robotAnimCfgBase,
                      const RobotOutInterface &interface) {
  _robotFieldMarkers = robotCfg.robotFieldMarkers;
  _fieldMarker = robotCfg.fieldMarker;

  if (ErrorCode::SUCCESS !=
      RobotInitHelper::init(initialState, robotAnimCfgBase, interface, *this)) {
    LOGERR("Error, RobotInitHelper::init() failed");
    return ErrorCode::FAILURE;
  }

  onInitEnd();
  return ErrorCode::SUCCESS;
}

void Robot::deinit() {
  if (_outInterface.collisionWatcher) {
    _outInterface.collisionWatcher->unregisterObject(_collisionObjHandle);
  }
}

void Robot::draw() const {
  _animator.draw();
}

RobotState Robot::getState() const {
  return _state;
}

Point Robot::getAbsolutePos() const {
  return _animator.getAbsolutePos();
}

double Robot::getRotationAngle() const {
  return _animator.getRotationAngle();
}

SurroundingTiles Robot::getSurroundingTiles() const {
  const auto& fieldDescr = _outInterface.getFieldDescriptionCb();
  return RobotUtils::getSurroundingTiles(fieldDescr, _state);
}

void Robot::act(MoveType moveType) {
  switch (moveType) {
  case MoveType::FORWARD:
    move();
    break;
  case MoveType::ROTATE_LEFT:
    _animator.startRotAnim(_state.fieldPos, _state.dir, RotationDir::LEFT);
    break;
  case MoveType::ROTATE_RIGHT:
    _animator.startRotAnim(_state.fieldPos, _state.dir, RotationDir::RIGHT);
    break;
  default:
    LOGERR("Error, received unsupported moveType: %d", getEnumValue(moveType));
    break;
  }
}

void Robot::onMoveAnimEnd(Direction futureDir, const FieldPos &futurePos) {
  if (RobotFieldMarkers::ENABLED == _robotFieldMarkers) {
    _outInterface.resetFieldDataMarkerCb(_state.fieldPos);
    _outInterface.setFieldDataMarkerCb(futurePos, _fieldMarker);
  }
  _state.dir = futureDir;
  _state.fieldPos = futurePos;

  if (CollisionWatchStatus::ON == _currCollisionWatchStatus) {
    _currCollisionWatchStatus = CollisionWatchStatus::OFF;
    _outInterface.collisionWatcher->toggleWatchStatus(_collisionObjHandle,
        _currCollisionWatchStatus);
  }

  _outInterface.finishRobotActCb(_state, MoveOutcome::SUCCESS);
}

void Robot::onInitEnd() {
  _collisionObjHandle = _outInterface.collisionWatcher->registerObject(this,
      CollisionDamageImpact::YES);

  if (RobotFieldMarkers::ENABLED == _robotFieldMarkers) {
    _outInterface.setFieldDataMarkerCb(_state.fieldPos, _fieldMarker);
  }
}

void Robot::registerCollision([[maybe_unused]]const Rectangle &intersectRect,
                              CollisionDamageImpact impact) {
  //collision watch status will not be started in the case where
  //this is the currently moving object
  if (CollisionWatchStatus::ON == _currCollisionWatchStatus) {
    //this is a soft object (such as a coin). Don't stop the movement
    if (CollisionDamageImpact::NO == impact) {
      return; //nothing more to do
    }

    _currCollisionWatchStatus = CollisionWatchStatus::OFF;
    _outInterface.collisionWatcher->toggleWatchStatus(_collisionObjHandle,
        _currCollisionWatchStatus);

    //invoke instant collision callback if set.
    //this is to inform about the collision before the collision animation
    if (_outInterface.playerRobotDamageCollisionCb) {
      _outInterface.playerRobotDamageCollisionCb();
    }

    _animator.stopMoveAnim();
    _animator.startCollisionImpactAnim(RobotEndTurn::YES);
    return;
  }

  //collision watch status will not be changed in the case where
  //another moving object collides into this one, which is not moving
  _animator.startCollisionImpactAnim(RobotEndTurn::NO);
}

Rectangle Robot::getBoundary() const {
  return _animator.getBoundary();
}

void Robot::move() {
  const auto futurePos = FieldUtils::getAdjacentPos(_state.dir,
      _state.fieldPos);
  if (FieldUtils::isInsideField(futurePos,
      _outInterface.getFieldDescriptionCb())) {
    _currCollisionWatchStatus = CollisionWatchStatus::ON;
    _outInterface.collisionWatcher->toggleWatchStatus(_collisionObjHandle,
        _currCollisionWatchStatus);
  } else {
    _animator.startWallCollisionTimer();
  }

  _animator.startMoveAnim(_state.fieldPos, _state.dir, futurePos);
}

void Robot::onCollisionImpactAnimEnd(RobotEndTurn status) {
  if (RobotEndTurn::YES == status) {
    _outInterface.finishRobotActCb(_state, MoveOutcome::COLLISION);
  }
}

void Robot::onCollisionImpact() {
  // only player robot should report damage callback
  if (_outInterface.playerDamageCb) {
    constexpr auto damage = 45;
    _outInterface.playerDamageCb(damage);
  }
}

