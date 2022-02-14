//Corresponding header
#include "robo_collector_gui/entities/robot/Robot.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "robo_common/field/FieldUtils.h"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/helpers/CollisionWatcher.h"

int32_t Robot::init(const RobotConfig &cfg,
                    const RobotAnimatorConfigBase &robotAnimCfgBase,
                    const RobotOutInterface &interface) {
  _state = cfg;

  if (SUCCESS != initOutInterface(interface)) {
    LOGERR("Error, initOutInterface() failed");
    return FAILURE;
  }

  if (SUCCESS != initRobotAnimator(robotAnimCfgBase)) {
    LOGERR("Error, initOutInterface() failed");
    return FAILURE;
  }

  onInitEnd();
  return SUCCESS;
}

void Robot::deinit() {
  if (_outInterface.collisionWatcher) {
    _outInterface.collisionWatcher->unregisterObject(_collisionObjHandle);
  }
}

void Robot::draw() const {
  _animator.draw();
}

FieldPos Robot::getFieldPos() const {
  return _state.fieldPos;
}

Direction Robot::getDirection() const {
  return _state.dir;
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
  _outInterface.resetFieldDataMarkerCb(_state.fieldPos);
  _state.dir = futureDir;
  _state.fieldPos = futurePos;
  _outInterface.setFieldDataMarkerCb(futurePos, _state.fieldMarker);

  if (CollisionWatchStatus::ON == _currCollisionWatchStatus) {
    _currCollisionWatchStatus = CollisionWatchStatus::OFF;
    _outInterface.collisionWatcher->toggleWatchStatus(_collisionObjHandle,
        _currCollisionWatchStatus);
  }

  _outInterface.finishRobotActCb(_state.robotId);
}

int32_t Robot::initOutInterface(const RobotOutInterface &interface) {
  _outInterface = interface;

  if (nullptr == _outInterface.playerDamageCb) {
    LOGERR("Error, nullptr provided for playerDamageCb");
    return FAILURE;
  }

  // only player robot should report damage callback
  if (Defines::PLAYER_ROBOT_IDX != _state.robotId) {
    _outInterface.playerDamageCb = nullptr;
  }

  if (nullptr == _outInterface.setFieldDataMarkerCb) {
    LOGERR("Error, nullptr provided for setFieldDataMarkerCb");
    return FAILURE;
  }

  if (nullptr == _outInterface.resetFieldDataMarkerCb) {
    LOGERR("Error, nullptr provided for resetFieldDataMarkerCb");
    return FAILURE;
  }

  if (nullptr == _outInterface.finishRobotActCb) {
    LOGERR("Error, nullptr provided for finishRobotActCb");
    return FAILURE;
  }

  if (nullptr == _outInterface.getFieldDataCb) {
    LOGERR("Error, nullptr provided for getFieldDataCb");
    return FAILURE;
  }

  if (nullptr == _outInterface.collisionWatcher) {
    LOGERR("Error, nullptr provided for collisionWatcher");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t Robot::initRobotAnimator(
    const RobotAnimatorConfigBase &robotAnimCfgBase) {
  using namespace std::placeholders;

  RobotAnimatorConfig cfg;
  cfg.baseCfg = robotAnimCfgBase;
  cfg.startDir = _state.dir;
  cfg.startPos = _state.fieldPos;
  cfg.onMoveAnimEndCb = std::bind(&Robot::onMoveAnimEnd, this, _1, _2);
  cfg.collisionImpactAnimEndCb =
      std::bind(&Robot::onCollisionImpactAnimEnd, this, _1);
  cfg.collisionImpactCb = std::bind(&Robot::onCollisionImpact, this);
  cfg.getRobotFieldPosCb = std::bind(&Robot::getFieldPos, this);

  if (SUCCESS != _animator.init(cfg)) {
    LOGERR("Error, RobotAnimator.init() failed");
    return FAILURE;
  }

  return SUCCESS;
}

void Robot::onInitEnd() {
  _collisionObjHandle = _outInterface.collisionWatcher->registerObject(this,
      CollisionDamageImpact::YES);

  _outInterface.setFieldDataMarkerCb(_state.fieldPos, _state.fieldMarker);
}

void Robot::registerCollision([[maybe_unused]]const Rectangle &intersectRect,
                              CollisionDamageImpact impact) {
  //collision watch status will not be started in the case where
  //this is the currently moving object
  if (CollisionWatchStatus::ON == _currCollisionWatchStatus) {
    _currCollisionWatchStatus = CollisionWatchStatus::OFF;
    _outInterface.collisionWatcher->toggleWatchStatus(_collisionObjHandle,
        _currCollisionWatchStatus);

    //this is a soft object (such as coin). Don't stop the movement
    if (CollisionDamageImpact::NO == impact) {
      return; //nothing more to do
    }

    _animator.stopMoveAnim();
    _animator.startCollisionImpactAnim(RobotEndTurn::YES);
    return;
  }

  //collision watch status will not be started in the case where
  //another moving object collides into this one, which is not moving
  _animator.startCollisionImpactAnim(RobotEndTurn::NO);
}

Rectangle Robot::getBoundary() const {
  return _animator.getBoundary();
}

void Robot::move() {
  const auto futurePos = FieldUtils::getAdjacentPos(_state.dir,
      _state.fieldPos);
  if (FieldUtils::isInsideField(futurePos)) {
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
    _outInterface.finishRobotActCb(_state.robotId);
  }
}

void Robot::onCollisionImpact() {
  // only player robot should report damage callback
  if (_outInterface.playerDamageCb) {
    constexpr auto damage = 45;
    _outInterface.playerDamageCb(damage);
  }
}

