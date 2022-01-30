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

int32_t Robot::init(const RobotConfig &cfg,
                    const RobotOutInterface &interface) {
  if (SUCCESS != initConfig(cfg)) {
    LOGERR("Error, initConfig() failed");
    return FAILURE;
  }

  if (SUCCESS != initOutInterface(interface)) {
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
  _robotImg.draw();
}

FieldPos Robot::getFieldPos() const {
  return _cfg.fieldPos;
}

Direction Robot::getDirection() const {
  return _cfg.dir;
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
  _outInterface.resetFieldDataMarkerCb(_cfg.fieldPos);
  _cfg.dir = futureDir;
  _cfg.fieldPos = futurePos;
  _outInterface.setFieldDataMarkerCb(futurePos, _cfg.fieldMarker);

  if (CollisionWatchStatus::ON == _currCollisionWatchStatus) {
    LOGB("Switching CollisionWatchStatus::OFF for robotId: %d", _cfg.robotId);
    _currCollisionWatchStatus = CollisionWatchStatus::OFF;
    _outInterface.collisionWatcher->toggleWatchStatus(
          _collisionObjHandle, _currCollisionWatchStatus);
  }

  _outInterface.finishRobotActCb(_cfg.robotId);
}

int32_t Robot::initConfig(const RobotConfig &cfg) {
  _cfg = cfg;

  _robotImg.create(_cfg.rsrcId);
  _robotImg.setPosition(FieldUtils::getAbsPos(_cfg.fieldPos));
  _robotImg.setFrame(_cfg.frameId);
  _robotImg.setPredefinedRotationCenter(RotationCenterType::ORIG_CENTER);
  _robotImg.setRotation(RobotUtils::getRotationDegFromDir(_cfg.dir));

  if (SUCCESS != _animEndCb.init(std::bind(&Robot::onMoveAnimEnd, this,
      std::placeholders::_1, std::placeholders::_2))) {
    LOGERR("Error, _animEndCb.init() failed");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t Robot::initOutInterface(const RobotOutInterface &interface) {
  _outInterface = interface;

  if (nullptr == _outInterface.playerDamageCb) {
    LOGERR("Error, nullptr provided for playerDamageCb");
    return FAILURE;
  }

  // only player robot should report damage callback
  if (Defines::PLAYER_ROBOT_IDX != _cfg.robotId) {
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

void Robot::onInitEnd() {
  _collisionObjHandle =_outInterface.collisionWatcher->registerObject(
      this, CollisionDamageImpact::YES);

  _outInterface.setFieldDataMarkerCb(_cfg.fieldPos, _cfg.fieldMarker);
}

void Robot::registerCollision([[maybe_unused]]const Rectangle& intersectRect,
                              CollisionDamageImpact impact) {
  LOGC("Received registerCollision for robotId: %d", _cfg.robotId);
  //collision watch status will not be started in the case where
  //this is the currently moving object
  if (CollisionWatchStatus::ON == _currCollisionWatchStatus) {
    LOGB("Switching CollisionWatchStatus::OFF for robotId: %d", _cfg.robotId);
    _currCollisionWatchStatus = CollisionWatchStatus::OFF;
    _outInterface.collisionWatcher->toggleWatchStatus(
          _collisionObjHandle, _currCollisionWatchStatus);

    //this is a soft object (such as coin). Don't stop the movement
    if (CollisionDamageImpact::NO == impact) {
      return; //nothing more to do
    }

    LOGM("Handle collision from registerCollision CollisionWatchStatus::ON "
        "for robotId: %d", _cfg.robotId);
    handleDamageImpactCollision();
    _outInterface.finishRobotActCb(_cfg.robotId);
    return;
  }

  //collision watch status will not be started in the case where
  //another moving object collides into this one, which is not moving
  //TODO invoke on collisionAnimEnd
  LOGM("Handle collision from registerCollision CollisionWatchStatus::OFF "
      "for robotId: %d", _cfg.robotId);
  handleDamageImpactCollision();
}

Rectangle Robot::getBoundary() const {
  return _robotImg.getImageRect();
}

void Robot::onTimeout(const int32_t timerId) {
  if (timerId == _cfg.wallCollisionAnimTimerId) {
    LOGM("Handle collision from onTimeout wall timer");
    //TODO invoke on collisionAnimEnd
    handleDamageImpactCollision();
    _outInterface.finishRobotActCb(_cfg.robotId);
  } else {
    LOGERR("Error, receive unsupported timerId: %d", timerId);
  }
}

void Robot::move() {
  const auto futurePos = FieldUtils::getAdjacentPos(_cfg.dir, _cfg.fieldPos);
  if (FieldUtils::isInsideField(futurePos)) {
    LOGB("Switching CollisionWatchStatus::ON for robotId: %d", _cfg.robotId);
    _currCollisionWatchStatus = CollisionWatchStatus::ON;
    _outInterface.collisionWatcher->toggleWatchStatus(
        _collisionObjHandle, _currCollisionWatchStatus);
  } else {
    constexpr auto timerInterval = 200;
    startTimer(
        timerInterval, _cfg.wallCollisionAnimTimerId, TimerType::ONESHOT);
  }

  startMoveAnim(futurePos);
}

void Robot::handleDamageImpactCollision() {
  LOGY("handling collision for robotId: %d", _cfg.robotId);

  _animEndCb.setCbStatus(RobotAnimEndCbReport::DISABLE);
  _moveAnim.stop();

  _robotImg.setPosition(FieldUtils::getAbsPos(_cfg.fieldPos));

  if (_outInterface.playerDamageCb) {
    LOGY("dealing damage for robotId: %d", _cfg.robotId);
    constexpr auto damage = 40;
    _outInterface.playerDamageCb(damage);
  }
}

void Robot::startMoveAnim(FieldPos futurePos) {
  const auto cfg = generateAnimBaseConfig();
  constexpr auto numberOfSteps = 20;
  const auto futureAbsPos = FieldUtils::getAbsPos(futurePos);
  _animEndCb.setAnimEndData(_cfg.dir, futurePos);
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
  const auto futureDir =
      RobotUtils::getDirAfterRotation(_cfg.dir, isLeftRotation);
  _animEndCb.setAnimEndData(futureDir, _cfg.fieldPos);
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
  cfg.startPos = FieldUtils::getAbsPos(_cfg.fieldPos);
  cfg.animDirection = AnimDir::FORWARD;
  cfg.timerId = _cfg.moveAnimTimerId;
  cfg.timerInterval = 20;
  cfg.isTimerPauseble = true;
  cfg.animImageType = AnimImageType::EXTERNAL;
  cfg.externalImage = &_robotImg;

  return cfg;
}

