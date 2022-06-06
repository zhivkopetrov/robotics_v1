//Corresponding header
#include "robo_common/layout/entities/robot/animation/RobotAnimator.h"

//System headers
#include <cmath>

//Other libraries headers
#include "robo_common/layout/field/FieldUtils.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_common/layout/entities/robot/helpers/RobotUtils.h"

namespace {
constexpr auto TOTAL_COLLISION_ANIM_STEPS = 32;
constexpr auto TARGET_DAMAGE_MARKER_ANIM_X = 1270;
constexpr auto TARGET_DAMAGE_MARKER_ANIM_Y = 397;
}

ErrorCode RobotAnimator::init(const RobotAnimatorConfig &cfg,
                              const RobotAnimatorOutInterface &outInterface) {
  if (ErrorCode::SUCCESS != initOutInterface(outInterface)) {
    LOGERR("Error, initOutInterface() failed");
    return ErrorCode::FAILURE;
  }

  const auto onPlayerAnimEndCb = std::bind(
      &RobotAnimator::onPlayerDamageAnimEnd, this);
  if (ErrorCode::SUCCESS != _playerDamageAnimEndCb.init(onPlayerAnimEndCb)) {
    LOGERR("Error, playerDamageAnimEndCb.init() failed");
    return ErrorCode::FAILURE;
  }

  _robotId = cfg.baseCfg.robotId;
  _moveAnimTimerId = cfg.baseCfg.moveAnimTimerId;
  _rotateAnimTimerId = cfg.baseCfg.rotateAnimTimerId;
  _robotCollisionAnimTimerId = cfg.baseCfg.robotCollisionAnimTimerId;
  _robotDamageAnimTimerId = cfg.baseCfg.robotDamageAnimTimerId;
  _damageMarkerRsrcId = cfg.baseCfg.damageMarkerRsrcId;

  _robotImg.create(cfg.baseCfg.robotRsrcId);
  const auto robotAbsPos = FieldUtils::getAbsPos(cfg.startPos,
      _outInterface.getFieldDescriptionCb());
  _robotImg.setPosition(robotAbsPos);
  _robotImg.setFrame(cfg.baseCfg.frameId);
  _robotImg.activateScaling();
  _robotImg.setScaledWidth(cfg.baseCfg.width);
  _robotImg.setScaledHeight(cfg.baseCfg.height);
  _robotImg.setPredefinedRotationCenter(RotationCenterType::SCALED_CENTER);
  _robotImg.setRotation(RobotUtils::getRotationDegFromDir(cfg.startDir));

  constexpr auto divisor = 80; //chosen to visually fit
  const auto step = cfg.baseCfg.width / divisor;
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

  configurePlayerDamageAnim();
  _playerDamageAnim.hideAnimation();

  return ErrorCode::SUCCESS;
}

void RobotAnimator::draw() const {
  _robotImg.draw();
  _playerDamageAnim.draw();
}

void RobotAnimator::startMoveAnim(const FieldPos &currPos, Direction currDir,
                                  const FieldPos &futurePos) {
  const auto cfg = generateAnimBaseConfig(currPos, AnimationType::MOVE);
  constexpr auto numberOfSteps = 20;
  const auto futureAbsPos = FieldUtils::getAbsPos(futurePos,
      _outInterface.getFieldDescriptionCb());
  _animEndCb.setAnimEndData(currDir, futurePos);
  _animEndCb.setCbStatus(RobotAnimEndCbReport::ENABLE);

  if (ErrorCode::SUCCESS !=
      _moveAnim.configure(cfg, futureAbsPos, numberOfSteps,
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
  const auto cfg = generateAnimBaseConfig(currPos, AnimationType::ROTATE);
  const auto angleSign = (RotationDir::LEFT == rotDir) ? -1.0 : 1.0;
  const auto rotAngleStep = 4.5 * angleSign;
  const auto totalRotAngle = 90.0 * angleSign;
  const auto rotCenter = _robotImg.getPredefinedRotationCenter(
      RotationCenterType::SCALED_CENTER);
  const auto futureDir = RobotUtils::getDirAfterRotation(currDir, rotDir);
  _animEndCb.setAnimEndData(futureDir, currPos);
  _animEndCb.setCbStatus(RobotAnimEndCbReport::ENABLE);

  if (ErrorCode::SUCCESS !=
      _rotAnim.configure(cfg, rotAngleStep, &_animEndCb, rotCenter,
      PosAnimType::ONE_DIRECTIONAL, AnimType::FINITE, totalRotAngle)) {
    LOGERR("Error in rotAnim.configure() for rsrcId: %#16lX", cfg.rsrcId);
    return;
  }

  _rotAnim.start();
}

void RobotAnimator::stopRotAnim() {
  _animEndCb.setCbStatus(RobotAnimEndCbReport::DISABLE);
  _rotAnim.stop();
}

void RobotAnimator::startCollisionImpactAnim(RobotEndTurn status) {
  _collisionAnimOutcome = status;
  _collisionAnimStep = 0;
  _currCollisionAnimFrame = 0;
  constexpr auto timerInterval = 40;
  startTimer(timerInterval, _robotCollisionAnimTimerId, TimerType::PULSE);

  if (RoboCommonDefines::PLAYER_ROBOT_IDX == _robotId) {
    configurePlayerDamageAnim();
    _playerDamageAnim.start();
  }
}

void RobotAnimator::cancelMove() {
  if (_moveAnim.isAnimationActive()) {
    stopMoveAnim();
    rollbackRobotState();
  } else if (_rotAnim.isAnimationActive()) {
    stopRotAnim();
    rollbackRobotState();
  }
}

Rectangle RobotAnimator::getBoundary() const {
  return _robotImg.getScaledRect();
}

Point RobotAnimator::getAbsolutePos() const {
  return _robotImg.getPosition();
}

double RobotAnimator::getRotationAngle() const {
  return _robotImg.getRotation();
}

ErrorCode RobotAnimator::initOutInterface(
    const RobotAnimatorOutInterface &outInterface) {
  _outInterface = outInterface;
  if (ErrorCode::SUCCESS != _animEndCb.init(_outInterface.onMoveAnimEndCb)) {
    LOGERR("Error, _animEndCb.init() failed");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.collisionImpactAnimEndCb) {
    LOGERR("Error, nullptr provided for collisionImpactAnimEndCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.collisionImpactCb) {
    LOGERR("Error, nullptr provided for collisionImpactCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.getRobotStateCb) {
    LOGERR("Error, nullptr provided for GetRobotStateCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.getFieldDescriptionCb) {
    LOGERR("Error, nullptr provided for GetFieldDescriptionCb");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

void RobotAnimator::onTimeout(const int32_t timerId) {
  if (timerId == _robotCollisionAnimTimerId) {
    processCollisionAnim();
  } else {
    LOGERR("Error, receive unsupported timerId: %d", timerId);
  }
}

void RobotAnimator::configurePlayerDamageAnim() {
  AnimBaseConfig cfg;
  cfg.rsrcId = _damageMarkerRsrcId;
  cfg.startPos = _robotImg.getPosition();
  cfg.animDirection = AnimDir::FORWARD;
  cfg.timerId = _robotDamageAnimTimerId;
  cfg.timerInterval = 20;
  cfg.isTimerPauseble = true;

  const Point endPos = Point(TARGET_DAMAGE_MARKER_ANIM_X,
      TARGET_DAMAGE_MARKER_ANIM_Y);
  const auto deltaX = endPos.x - cfg.startPos.x;
  const auto deltaY = endPos.y - cfg.startPos.y;
  const auto pixelDistance = static_cast<int32_t>(sqrt(
      (deltaX * deltaX) + (deltaY * deltaY)));
  constexpr auto pixelsPerStep = 25;
  const auto numberOfSteps = pixelDistance / pixelsPerStep;

  if (ErrorCode::SUCCESS !=
      _playerDamageAnim.configure(cfg, endPos, numberOfSteps,
      &_playerDamageAnimEndCb, PosAnimType::ONE_DIRECTIONAL)) {
    LOGERR("playerDamagaAnim.configure() failed");
    return;
  }
  _playerDamageAnim.showAnimation();
}

void RobotAnimator::onPlayerDamageAnimEnd() {
  _playerDamageAnim.hideAnimation();
  //invoke the direct collision cb to draw the health decrease while playing
  //the robot collision anim
  _outInterface.collisionImpactCb();
}

AnimBaseConfig RobotAnimator::generateAnimBaseConfig(
    const FieldPos &currPos, AnimationType animationType) {
  AnimBaseConfig cfg;
  cfg.rsrcId = _robotImg.getRsrcId();
  cfg.startPos =
      FieldUtils::getAbsPos(currPos, _outInterface.getFieldDescriptionCb());
  cfg.animDirection = AnimDir::FORWARD;
  cfg.timerId = (AnimationType::MOVE == animationType) ?
      _moveAnimTimerId : _rotateAnimTimerId;
  cfg.timerInterval = 20;
  cfg.isTimerPauseble = true;
  cfg.animImageType = AnimImageType::EXTERNAL;
  cfg.externalImage = &_robotImg;

  return cfg;
}

void RobotAnimator::processCollisionAnim() {
  if (TOTAL_COLLISION_ANIM_STEPS == _collisionAnimStep) {
    stopTimer(_robotCollisionAnimTimerId);
    rollbackRobotState();
    _outInterface.collisionImpactAnimEndCb(_collisionAnimOutcome);
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

void RobotAnimator::rollbackRobotState() {
  const RobotState robotState = _outInterface.getRobotStateCb();
  const Point robotAbsPos = FieldUtils::getAbsPos(robotState.fieldPos,
      _outInterface.getFieldDescriptionCb());
  _robotImg.setPosition(robotAbsPos);
  _robotImg.setRotation(RobotUtils::getRotationDegFromDir(robotState.dir));
}

