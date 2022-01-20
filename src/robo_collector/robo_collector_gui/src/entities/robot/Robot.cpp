//Corresponding header
#include "robo_collector_gui/entities/robot/Robot.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/data_type/EnumClassUtils.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/field/FieldUtils.h"
#include "robo_collector_gui/entities/robot/RobotUtils.h"

int32_t Robot::init(const RobotCfg &cfg) {
  using namespace std::placeholders;

  if (nullptr == cfg._collisionCb) {
    LOGERR("Error, nullptr provided for RobotCfg collisionCb");
    return FAILURE;
  }
  _collisionCb = cfg._collisionCb;
  _animTimerId = cfg.animTimerId;

  _robotImg.create(cfg.rsrcId);
  _robotImg.setPosition(FieldUtils::getAbsPos(cfg.fieldPos));
  _robotImg.setFrame(cfg.frameId);

  _fieldPos = cfg.fieldPos;

  if (SUCCESS != _animEndCb.init(
          std::bind(&Robot::setMoveData, this, _1, _2))) {
    LOGERR("Error, _animEndCb.init() failed");
    return FAILURE;
  }

  return SUCCESS;
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

void Robot::setMoveData(Direction futureDir, const FieldPos &futurePos) {
  _dir = futureDir;
  _fieldPos = futurePos;
}

void Robot::move() {
  const auto futurePos = FieldUtils::getAdjacentPos(_dir, _fieldPos);
  if (FieldUtils::isInsideField(futurePos)) {
    startPosAnim(futurePos);
  } else {
    constexpr auto damage = 10;
    _collisionCb(damage);
  }
}

void Robot::startPosAnim(FieldPos futurePos) {
  const auto cfg = generateAnimBaseConfig();
  constexpr auto numberOfSteps = 40;
  const auto futureAbsPos = FieldUtils::getAbsPos(futurePos);
  _animEndCb.setAnimEndData(_dir, futurePos);

  if (SUCCESS != _posAnim.configure(cfg, futureAbsPos, numberOfSteps,
          &_animEndCb, PosAnimType::ONE_DIRECTIONAL)) {
    LOGERR("Error in posAnim.configure() for rsrcId: %#16lX", cfg.rsrcId);
    return;
  }

  _posAnim.start();
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

