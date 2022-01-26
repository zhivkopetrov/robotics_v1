//Corresponding header
#include "robo_collector_gui/entities/robot/Robot.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/data_type/EnumClassUtils.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/helpers/CollisionWatcher.h"
#include "robo_collector_gui/field/FieldUtils.h"
#include "robo_collector_gui/entities/robot/RobotUtils.h"

int32_t Robot::init(const RobotCfg &cfg) {
  if (nullptr == cfg.playerDamageCb) {
    LOGERR("Error, nullptr provided for RobotCfg collisionCb");
    return FAILURE;
  }

  // only player robot should report damage callback
  _playerDamageCb =
      (Defines::PLAYER_ROBOT_IDX == cfg.robotId) ? cfg.playerDamageCb : nullptr;

  if (nullptr == cfg.setFieldDataMarkerCb) {
    LOGERR("Error, nullptr provided for RobotCfg setFieldDataMarkerCb");
    return FAILURE;
  }
  _setFieldDataMarkerCb = cfg.setFieldDataMarkerCb;

  if (nullptr == cfg.resetFieldDataMarkerCb) {
    LOGERR("Error, nullptr provided for RobotCfg resetFieldDataMarkerCb");
    return FAILURE;
  }
  _resetFieldDataMarkerCb = cfg.resetFieldDataMarkerCb;

  if (nullptr == cfg.finishRobotActCb) {
    LOGERR("Error, nullptr provided for RobotCfg finishRobotActCb");
    return FAILURE;
  }
  _finishRobotActCb = cfg.finishRobotActCb;

  if (nullptr == cfg.getFieldDataCb) {
    LOGERR("Error, nullptr provided for RobotCfg getFieldDataCb");
    return FAILURE;
  }
  _getFieldDataCb = cfg.getFieldDataCb;

  _fieldPos = cfg.fieldPos;
  _dir = cfg.initialDir;
  _robotId = cfg.robotId;

  _moveAnimTimerId = cfg.moveAnimTimerId;
  _wallCollisionAnimTimerId = cfg.wallCollisionAnimTimerId;
  _robotImg.create(cfg.rsrcId);
  _robotImg.setPosition(FieldUtils::getAbsPos(cfg.fieldPos));
  _robotImg.setFrame(cfg.frameId);
  _robotImg.setPredefinedRotationCenter(RotationCenterType::ORIG_CENTER);
  _robotImg.setRotation(RobotUtils::getRotationDegFromDir(cfg.initialDir));

  _selfFieldMarker = cfg.fieldMarker;
  _enemyFieldMarker = cfg.enemyFieldMarker;

  if (SUCCESS != _animEndCb.init(std::bind(&Robot::onMoveAnimEnd, this,
      std::placeholders::_1, std::placeholders::_2))) {
    LOGERR("Error, _animEndCb.init() failed");
    return FAILURE;
  }

  if (nullptr == cfg.collisionWatcher) {
    LOGERR("Error, nullptr provided for collisionWatcher");
    return FAILURE;
  }
  _collisionWatcher = cfg.collisionWatcher;
  _collisionObjHandle =
      cfg.collisionWatcher->registerObject(this, CollisionDamageImpact::YES);

  _setFieldDataMarkerCb(_fieldPos, _selfFieldMarker);

  return SUCCESS;
}

void Robot::deinit() {
  if (_collisionWatcher) {
    _collisionWatcher->unregisterObject(_collisionObjHandle);
  }
}

void Robot::draw() const {
  _robotImg.draw();
}

FieldPos Robot::getFieldPos() const {
  return _fieldPos;
}

Direction Robot::getDirection() const {
  return _dir;
}

void Robot::act(MoveType moveType) {
  switch (moveType) {
  case MoveType::FORWARD:
    move();
    break;

  case MoveType::ROTATE_LEFT:
    startRotAnim(true /*isLeftRotation*/);
    break;
  case MoveType::ROTATE_RIGHT:
    startRotAnim(false /*isLeftRotation*/);
    break;

  default:
    LOGERR("Error, received unsupported moveType: %d", getEnumValue(moveType));
    break;
  }
}

void Robot::onMoveAnimEnd(Direction futureDir, const FieldPos &futurePos) {
  _resetFieldDataMarkerCb(_fieldPos);
  _dir = futureDir;
  _fieldPos = futurePos;
  _setFieldDataMarkerCb(futurePos, _selfFieldMarker);

  if (CollisionWatchStatus::ON == _currCollisionWatchStatus) {
    LOGB("Switching CollisionWatchStatus::OFF for robotId: %d", _robotId);
    _currCollisionWatchStatus = CollisionWatchStatus::OFF;
    _collisionWatcher->toggleWatchStatus(
          _collisionObjHandle, _currCollisionWatchStatus);
  }

  _finishRobotActCb(_robotId);
}

void Robot::registerCollision([[maybe_unused]]const Rectangle& intersectRect,
                              CollisionDamageImpact impact) {
  LOGC("Received registerCollision for robotId: %d", _robotId);
  //collision watch status will not be started in the case where
  //this is the currently moving object
  if (CollisionWatchStatus::ON == _currCollisionWatchStatus) {
    LOGB("Switching CollisionWatchStatus::OFF for robotId: %d", _robotId);
    _currCollisionWatchStatus = CollisionWatchStatus::OFF;
    _collisionWatcher->toggleWatchStatus(
          _collisionObjHandle, _currCollisionWatchStatus);

    //this is a soft object (such as coin). Don't stop the movement
    if (CollisionDamageImpact::NO == impact) {
      return; //nothing more to do
    }

    LOGM("Handle collision from registerCollision CollisionWatchStatus::ON "
        "for robotId: %d", _robotId);
    handleDamageImpactCollision();
    _finishRobotActCb(_robotId);
    return;
  }

  //collision watch status will not be started in the case where
  //another moving object collides into this one, which is not moving
  //TODO invoke on collisionAnimEnd
  LOGM("Handle collision from registerCollision CollisionWatchStatus::OFF "
      "for robotId: %d", _robotId);
  handleDamageImpactCollision();
}

Rectangle Robot::getBoundary() const {
  return _robotImg.getImageRect();
}

void Robot::onTimeout(const int32_t timerId) {
  if (timerId == _wallCollisionAnimTimerId) {
    LOGM("Handle collision from onTimeout wall timer");
    //TODO invoke on collisionAnimEnd
    handleDamageImpactCollision();
    _finishRobotActCb(_robotId);
  } else {
    LOGERR("Error, receive unsupported timerId: %d", timerId);
  }
}

void Robot::move() {
  const auto futurePos = FieldUtils::getAdjacentPos(_dir, _fieldPos);
  if (FieldUtils::isInsideField(futurePos)) {
    LOGB("Switching CollisionWatchStatus::ON for robotId: %d", _robotId);
    _currCollisionWatchStatus = CollisionWatchStatus::ON;
    _collisionWatcher->toggleWatchStatus(
        _collisionObjHandle, _currCollisionWatchStatus);
  } else {
    constexpr auto timerInterval = 200;
    startTimer(timerInterval, _wallCollisionAnimTimerId, TimerType::ONESHOT);
  }

  startMoveAnim(futurePos);
}

void Robot::handleDamageImpactCollision() {
  LOGY("handling collision for robotId: %d", _robotId);

  _animEndCb.setCbStatus(RobotAnimEndCbReport::DISABLE);
  _moveAnim.stop();

  _robotImg.setPosition(FieldUtils::getAbsPos(_fieldPos));

  if (_playerDamageCb) {
    LOGY("dealing damage for robotId: %d", _robotId);
    constexpr auto damage = 20;
    _playerDamageCb(damage);
  }
}

void Robot::startMoveAnim(FieldPos futurePos) {
  const auto cfg = generateAnimBaseConfig();
  constexpr auto numberOfSteps = 20;
  const auto futureAbsPos = FieldUtils::getAbsPos(futurePos);
  _animEndCb.setAnimEndData(_dir, futurePos);
  _animEndCb.setCbStatus(RobotAnimEndCbReport::ENABLE);

  if (SUCCESS != _moveAnim.configure(cfg, futureAbsPos, numberOfSteps,
          &_animEndCb, PosAnimType::ONE_DIRECTIONAL)) {
    LOGERR("Error in posAnim.configure() for rsrcId: %#16lX", cfg.rsrcId);
    return;
  }

  _moveAnim.start();
}

void Robot::startRotAnim(bool isLeftRotation) {
  const auto cfg = generateAnimBaseConfig();
  const auto angleSign = isLeftRotation ? -1.0 : 1.0;
  const auto rotAngleStep = 4.5 * angleSign;
  const auto totalRotAngle = 90.0 * angleSign;
  const auto rotCenter = _robotImg.getPredefinedRotationCenter(
      RotationCenterType::ORIG_CENTER);
  const auto futureDir = RobotUtils::getDirAfterRotation(_dir, isLeftRotation);
  _animEndCb.setAnimEndData(futureDir, _fieldPos);
  _animEndCb.setCbStatus(RobotAnimEndCbReport::ENABLE);

  if (SUCCESS != _rotAnim.configure(cfg, rotAngleStep, &_animEndCb, rotCenter,
          PosAnimType::ONE_DIRECTIONAL, AnimType::FINITE, totalRotAngle)) {
    LOGERR("Error in rotAnim.configure() for rsrcId: %#16lX", cfg.rsrcId);
    return;
  }

  _rotAnim.start();
}

AnimBaseConfig Robot::generateAnimBaseConfig() {
  AnimBaseConfig cfg;
  cfg.rsrcId = _robotImg.getRsrcId();
  cfg.startPos = FieldUtils::getAbsPos(_fieldPos);
  cfg.animDirection = AnimDir::FORWARD;
  cfg.timerId = _moveAnimTimerId;
  cfg.timerInterval = 20;
  cfg.isTimerPauseble = true;
  cfg.animImageType = AnimImageType::EXTERNAL;
  cfg.externalImage = &_robotImg;

  return cfg;
}

