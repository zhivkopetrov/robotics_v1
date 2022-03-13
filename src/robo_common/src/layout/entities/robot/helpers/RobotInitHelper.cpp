//Corresponding header
#include "robo_common/layout/entities/robot/helpers/RobotInitHelper.h"

//System headers

//Other libraries headers
#include "utils/Log.h"

//Own components headers
#include "robo_common/layout/entities/robot/Robot.h"

ErrorCode RobotInitHelper::init(const RobotState &initialState,
                              const RobotAnimatorConfigBase &robotAnimCfgBase,
                              const RobotOutInterface &interface,
                              Robot &robot) {
  robot._state = initialState;

  if (ErrorCode::SUCCESS != initOutInterface(interface, robot)) {
    LOGERR("Error, initOutInterface() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initAnimator(robotAnimCfgBase, robot)) {
    LOGERR("Error, initAnimator() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode RobotInitHelper::initOutInterface(const RobotOutInterface &interface,
                                          Robot &robot) {
  robot._outInterface = interface;

  if (nullptr == robot._outInterface.playerDamageCb) {
    LOGERR("Error, nullptr provided for PlayerDamageCb");
    return ErrorCode::FAILURE;
  }

  // only player robot should report damage callback
  if (RoboCommonDefines::PLAYER_ROBOT_IDX != robot._state.robotId) {
    robot._outInterface.playerDamageCb = nullptr;
  }

  if (nullptr == robot._outInterface.setFieldDataMarkerCb) {
    LOGERR("Error, nullptr provided for setFieldDataMarkerCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == robot._outInterface.resetFieldDataMarkerCb) {
    LOGERR("Error, nullptr provided for resetFieldDataMarkerCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == robot._outInterface.finishRobotActCb) {
    LOGERR("Error, nullptr provided for finishRobotActCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == robot._outInterface.getFieldDescriptionCb) {
    LOGERR("Error, nullptr provided for GetFieldDescriptionCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == robot._outInterface.collisionWatcher) {
    LOGERR("Error, nullptr provided for collisionWatcher");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode RobotInitHelper::initAnimator(
    const RobotAnimatorConfigBase &robotAnimCfgBase, Robot &robot) {
  using namespace std::placeholders;

  RobotAnimatorConfig cfg;
  cfg.baseCfg = robotAnimCfgBase;
  cfg.startDir = robot._state.dir;
  cfg.startPos = robot._state.fieldPos;

  RobotAnimatorOutInterface interface;
  interface.onMoveAnimEndCb = std::bind(&Robot::onMoveAnimEnd, &robot, _1, _2);
  interface.collisionImpactAnimEndCb =
      std::bind(&Robot::onCollisionImpactAnimEnd, &robot, _1);
  interface.collisionImpactCb = std::bind(&Robot::onCollisionImpact, &robot);
  interface.getRobotStateCb = std::bind(&Robot::getState, &robot);
  interface.getFieldDescriptionCb = robot._outInterface.getFieldDescriptionCb;

  if (ErrorCode::SUCCESS != robot._animator.init(cfg, interface)) {
    LOGERR("Error, RobotAnimator.init() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

