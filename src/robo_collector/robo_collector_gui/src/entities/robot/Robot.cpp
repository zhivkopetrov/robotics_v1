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
  if (nullptr == cfg.collisionCb) {
    LOGERR("Error, nullptr provided for RobotCfg collisionCb");
    return FAILURE;
  }
  _collisionCb = cfg.collisionCb;

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

  _animTimerId = cfg.animTimerId;
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

FieldPos Robot::getFieldPos() const {
  return _fieldPos;
}

void Robot::onMoveAnimEnd(Direction futureDir, const FieldPos &futurePos) {
  _resetFieldDataMarkerCb(_fieldPos);
  _dir = futureDir;
  _fieldPos = futurePos;
  _setFieldDataMarkerCb(futurePos, _selfFieldMarker);

  if (CollisionWatchStatus::ON == _currCollisionWatchStatus) {
    _currCollisionWatchStatus = CollisionWatchStatus::OFF;
    _collisionWatcher->toggleWatchStatus(
          _collisionObjHandle, _currCollisionWatchStatus);
  }

  _finishRobotActCb();
}

void Robot::registerCollision([[maybe_unused]]const Rectangle& intersectRect,
                              CollisionDamageImpact impact) {
  //if collision watch status was started -> disable it
  //collision watch status will not be started in the case where
  //another object collides into this one
  if (CollisionWatchStatus::ON == _currCollisionWatchStatus) {
    _currCollisionWatchStatus = CollisionWatchStatus::OFF;
    _collisionWatcher->toggleWatchStatus(
          _collisionObjHandle, _currCollisionWatchStatus);
  }

  if (CollisionDamageImpact::NO == impact) {
    return; //nothing more to do
  }
}

Rectangle Robot::getBoundary() const {
  return _robotImg.getImageRect();
}

void Robot::move() {
  _currCollisionWatchStatus = CollisionWatchStatus::ON;
  _collisionWatcher->toggleWatchStatus(
      _collisionObjHandle, _currCollisionWatchStatus);
  const auto futurePos = FieldUtils::getAdjacentPos(_dir, _fieldPos);
  if (FieldUtils::isInsideField(futurePos)) {
    startMoveAnim(futurePos);
  } else {
    constexpr auto damage = 20;
    _collisionCb(damage);
    _finishRobotActCb();
  }
}

void Robot::startMoveAnim(FieldPos futurePos) {
  const auto cfg = generateAnimBaseConfig();
  constexpr auto numberOfSteps = 40;
  const auto futureAbsPos = FieldUtils::getAbsPos(futurePos);
  _animEndCb.setAnimEndData(_dir, futurePos);

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
  const auto rotAngleStep = 2.5 * angleSign;
  const auto totalRotAngle = 90.0 * angleSign;
  const auto rotCenter = _robotImg.getPredefinedRotationCenter(
      RotationCenterType::ORIG_CENTER);
  const auto futureDir = RobotUtils::getDirAfterRotation(_dir, isLeftRotation);
  _animEndCb.setAnimEndData(futureDir, _fieldPos);

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
  cfg.timerId = _animTimerId;
  cfg.timerInterval = 20;
  cfg.isTimerPauseble = true;
  cfg.animImageType = AnimImageType::EXTERNAL;
  cfg.externalImage = &_robotImg;

  return cfg;
}

