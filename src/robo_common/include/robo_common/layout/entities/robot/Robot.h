#ifndef ROBO_COMMON_ROBOT_H_
#define ROBO_COMMON_ROBOT_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"
#include "robo_common/helpers/CollisionObject.h"
#include "robo_common/layout/entities/robot/animation/RobotAnimator.h"

//Forward declarations
class CollisionWatcher;
class RobotInitHelper;

struct RobotOutInterface {
  PlayerDamageCb playerDamageCb;
  SetFieldDataMarkerCb setFieldDataMarkerCb;
  ResetFieldDataMarkerCb resetFieldDataMarkerCb;
  GetFieldDescriptionCb getFieldDescriptionCb;
  FinishRobotActCb finishRobotActCb;
  CollisionWatcher *collisionWatcher = nullptr;
};

struct RobotConfig {
  FieldPos fieldPos;
  int32_t robotId = 0;
  Direction dir = Direction::UP;
  char fieldMarker = '!';
  char enemyFieldMarker = '?';
};

class Robot final : public CollisionObject {
public:
  friend class RobotInitHelper;

  int32_t init(const RobotConfig &cfg,
               const RobotAnimatorConfigBase &robotAnimCfgBase,
               const RobotOutInterface &interface);

  void deinit();

  void draw() const;

  FieldPos getFieldPos() const;
  Direction getDirection() const;
  void act(MoveType moveType);

  void onMoveAnimEnd(Direction futureDir, const FieldPos &futurePos);
  void onCollisionImpactAnimEnd(RobotEndTurn status);
  void onCollisionImpact();

private:
  void onInitEnd();

  void registerCollision(const Rectangle &intersectRect,
                         CollisionDamageImpact impact) override;
  Rectangle getBoundary() const override;

  void move();

  RobotConfig _state;
  RobotOutInterface _outInterface;
  RobotAnimator _animator;
  CollisionWatchStatus _currCollisionWatchStatus = CollisionWatchStatus::OFF;
};

#endif /* ROBO_COMMON_ROBOT_H_ */
