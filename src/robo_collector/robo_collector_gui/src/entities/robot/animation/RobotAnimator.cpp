//Corresponding header
#include "robo_collector_gui/entities/robot/animation/RobotAnimator.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/field/FieldUtils.h"
#include "robo_collector_gui/entities/robot/RobotUtils.h"

namespace {
constexpr auto TOTAL_COLLISION_ANIM_STEPS = 24;
}

int32_t RobotAnimator::init(const RobotAnimatorConfig &cfg) {
  _moveAnimTimerId = cfg.baseCfg.moveAnimTimerId;
  _wallCollisionAnimTimerId = cfg.baseCfg.wallCollisionAnimTimerId;
  _robotCollisionAnimTimerId = cfg.baseCfg.robotCollisionAnimTimerId;

  _robotImg.create(cfg.baseCfg.rsrcId);
  _robotImg.setPosition(FieldUtils::getAbsPos(cfg.startPos));
  _robotImg.setFrame(cfg.baseCfg.frameId);
  _robotImg.setPredefinedRotationCenter(RotationCenterType::ORIG_CENTER);
  _robotImg.setRotation(RobotUtils::getRotationDegFromDir(cfg.startDir));

  constexpr auto step = 2;
  _collisionOffsets = {
      Point(-step, -step), //go up left
      Point(step, step),   //return center
      Point(step, step),   //go down right
      Point(-step, -step), //return center
      Point(step, -step),  //go up right
      Point(-step, step),  //return center
      Point(-step, step),  //go down left
      Point(step, -step),  //return center
  };

  if (SUCCESS != _animEndCb.init(cfg.onMoveAnimEndCb)) {
    LOGERR("Error, _animEndCb.init() failed");
    return FAILURE;
  }

  if (nullptr == cfg.collisionImpactAnimEndCb) {
    LOGERR("Error, nullptr provided for collisionImpactAnimEndCb");
    return FAILURE;
  }
  _collisionImpactAnimEndCb = cfg.collisionImpactAnimEndCb;

  if (nullptr == cfg.collisionImpactCb) {
    LOGERR("Error, nullptr provided for collisionImpactCb");
    return FAILURE;
  }
  _collisionImpactCb = cfg.collisionImpactCb;

  if (nullptr == cfg.getRobotFieldPosCb) {
    LOGERR("Error, nullptr provided for GetRobotFieldPosCb");
    return FAILURE;
  }
  _getRobotFieldPosCb = cfg.getRobotFieldPosCb;

  return SUCCESS;
}

void RobotAnimator::draw() const {
  _robotImg.draw();
}

void RobotAnimator::onTimeout(const int32_t timerId) {
  if (timerId == _wallCollisionAnimTimerId) {
    stopMoveAnim();
    startCollisionImpactAnim(RobotEndTurn::YES);
  } else if (timerId == _robotCollisionAnimTimerId) {
    processCollisionAnim();
  } else {
    LOGERR("Error, receive unsupported timerId: %d", timerId);
  }
}

void RobotAnimator::startMoveAnim(const FieldPos &currPos, Direction currDir,
                                  const FieldPos &futurePos) {
  const auto cfg = generateAnimBaseConfig(currPos);
  constexpr auto numberOfSteps = 20;
  const auto futureAbsPos = FieldUtils::getAbsPos(futurePos);
  _animEndCb.setAnimEndData(currDir, futurePos);
  _animEndCb.setCbStatus(RobotAnimEndCbReport::ENABLE);

  if (SUCCESS != _moveAnim.configure(cfg, futureAbsPos, numberOfSteps,
          &_animEndCb, PosAnimType::ONE_DIRECTIONAL)) {
    LOGERR("Error in posAnim.configure() for rsrcId: %#16lX", cfg.rsrcId);
    return;
  }

  _moveAnim.start();
}

void RobotAnimator::stopMoveAnim() {
  _animEndCb.setCbStatus(RobotAnimEndCbReport::DISABLE);
  _moveAnim.stop();
}

void RobotAnimator::startRotAnim(const FieldPos &currPos, Direction currDir,
                                 RotationDir rotDir) {
  const auto cfg = generateAnimBaseConfig(currPos);
  const auto angleSign = (RotationDir::LEFT == rotDir) ? -1.0 : 1.0;
  const auto rotAngleStep = 4.5 * angleSign;
  const auto totalRotAngle = 90.0 * angleSign;
  const auto rotCenter = _robotImg.getPredefinedRotationCenter(
      RotationCenterType::ORIG_CENTER);
  const auto futureDir = RobotUtils::getDirAfterRotation(currDir, rotDir);
  _animEndCb.setAnimEndData(futureDir, currPos);
  _animEndCb.setCbStatus(RobotAnimEndCbReport::ENABLE);

  if (SUCCESS != _rotAnim.configure(cfg, rotAngleStep, &_animEndCb, rotCenter,
          PosAnimType::ONE_DIRECTIONAL, AnimType::FINITE, totalRotAngle)) {
    LOGERR("Error in rotAnim.configure() for rsrcId: %#16lX", cfg.rsrcId);
    return;
  }

  _rotAnim.start();
}

void RobotAnimator::startWallCollisionTimer() {
  constexpr auto timerInterval = 80;
  startTimer(timerInterval, _wallCollisionAnimTimerId, TimerType::ONESHOT);
}

void RobotAnimator::startCollisionImpactAnim(RobotEndTurn status) {
  _collisionAnimOutcome = status;
  _collisionAnimStep = 0;
  _currCollisionAnimFrame = 0;
  constexpr auto timerInterval = 40;
  startTimer(timerInterval, _robotCollisionAnimTimerId, TimerType::PULSE);

  //invoke the direct collision cb to draw the health decrease while playing
  //the robot collision anim
  _collisionImpactCb();
}

Rectangle RobotAnimator::getBoundary() const {
  return _robotImg.getImageRect();
}

AnimBaseConfig RobotAnimator::generateAnimBaseConfig(const FieldPos &currPos) {
  AnimBaseConfig cfg;
  cfg.rsrcId = _robotImg.getRsrcId();
  cfg.startPos = FieldUtils::getAbsPos(currPos);
  cfg.animDirection = AnimDir::FORWARD;
  cfg.timerId = _moveAnimTimerId;
  cfg.timerInterval = 20;
  cfg.isTimerPauseble = true;
  cfg.animImageType = AnimImageType::EXTERNAL;
  cfg.externalImage = &_robotImg;

  return cfg;
}

void RobotAnimator::processCollisionAnim() {
  if (TOTAL_COLLISION_ANIM_STEPS == _collisionAnimStep) {
    stopTimer(_robotCollisionAnimTimerId);
    _robotImg.setPosition(FieldUtils::getAbsPos(_getRobotFieldPosCb()));
    _collisionImpactAnimEndCb(_collisionAnimOutcome);
    return;
  }

  auto pos = _robotImg.getPosition();
  pos += _collisionOffsets[_currCollisionAnimFrame];
  _robotImg.setPosition(pos);

  ++_currCollisionAnimFrame;
  if (COLLISION_ANIM_FRAMES == _currCollisionAnimFrame) {
    _currCollisionAnimFrame = 0;
  }

  ++_collisionAnimStep;
}

