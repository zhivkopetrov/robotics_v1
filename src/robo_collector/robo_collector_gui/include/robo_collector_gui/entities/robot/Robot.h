#ifndef ROBO_COLLECTOR_GUI_ROBOT_H_
#define ROBO_COLLECTOR_GUI_ROBOT_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "manager_utils/drawing/Image.h"
#include "manager_utils/drawing/animation/PositionAnimation.h"
#include "manager_utils/drawing/animation/RotationAnimation.h"
#include "manager_utils/time/TimerClient.h"

//Own components headers
#include "robo_collector_gui/helpers/CollisionObject.h"
#include "robo_collector_gui/entities/robot/animation/RobotAnimEndCb.h"
#include "robo_collector_gui/defines/RoboCollectorGuiFunctionalDefines.h"
#include "robo_collector_gui/field/FieldPos.h"

//Forward declarations
class CollisionWatcher;

struct RobotCfg {
  FieldPos fieldPos;
  uint64_t rsrcId = 0;
  int32_t robotId = 0;
  int32_t frameId = 0;
  int32_t moveAnimTimerId = 0;
  int32_t wallCollisionAnimTimerId = 0;
  Direction initialDir = Direction::UP;
  char fieldMarker = '!';
  char enemyFieldMarker = '?';

  PlayerDamageCb playerDamageCb;
  SetFieldDataMarkerCb setFieldDataMarkerCb;
  ResetFieldDataMarkerCb resetFieldDataMarkerCb;
  GetFieldDataCb getFieldDataCb;
  FinishRobotActCb finishRobotActCb;

  CollisionWatcher* collisionWatcher = nullptr;
};

class Robot final : public CollisionObject, public TimerClient {
public:
  int32_t init(const RobotCfg &cfg);

  void deinit();

  void draw() const;

  FieldPos getFieldPos() const;
  Direction getDirection() const;
  void act(MoveType moveType);

  void onMoveAnimEnd(Direction futureDir, const FieldPos &futurePos);

private:
  void registerCollision(const Rectangle& intersectRect,
                         CollisionDamageImpact impact) override;
  Rectangle getBoundary() const override;
  void onTimeout(const int32_t timerId) override;

  void move();

  void handleDamageImpactCollision();

  void startMoveAnim(FieldPos futurePos);
  void startRotAnim(bool isLeftRotation);
  AnimBaseConfig generateAnimBaseConfig();

  Image _robotImg;
  FieldPos _fieldPos;
  Direction _dir = Direction::UP;
  int32_t _robotId = 0;
  int32_t _moveAnimTimerId = 0;
  int32_t _wallCollisionAnimTimerId = 0;
  char _selfFieldMarker = '!';
  char _enemyFieldMarker = '?';

  PlayerDamageCb _playerDamageCb;
  SetFieldDataMarkerCb _setFieldDataMarkerCb;
  ResetFieldDataMarkerCb _resetFieldDataMarkerCb;
  GetFieldDataCb _getFieldDataCb;
  FinishRobotActCb _finishRobotActCb;

  PositionAnimation _moveAnim;
  RotationAnimation _rotAnim;
  RobotAnimEndCb _animEndCb;

  CollisionWatcher* _collisionWatcher = nullptr;
  CollisionWatchStatus _currCollisionWatchStatus = CollisionWatchStatus::OFF;
};

#endif /* ROBO_COLLECTOR_GUI_ROBOT_H_ */
