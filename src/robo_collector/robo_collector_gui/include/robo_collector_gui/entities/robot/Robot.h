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

struct RobotOutInterface {
  PlayerDamageCb playerDamageCb;
  SetFieldDataMarkerCb setFieldDataMarkerCb;
  ResetFieldDataMarkerCb resetFieldDataMarkerCb;
  GetFieldDataCb getFieldDataCb;
  FinishRobotActCb finishRobotActCb;
  CollisionWatcher* collisionWatcher = nullptr;
};

struct RobotConfig {
  FieldPos fieldPos;
  uint64_t rsrcId = 0;
  int32_t robotId = 0;
  int32_t frameId = 0;
  int32_t moveAnimTimerId = 0;
  int32_t wallCollisionAnimTimerId = 0;
  Direction dir = Direction::UP;
  char fieldMarker = '!';
  char enemyFieldMarker = '?';
};

class Robot final : public CollisionObject, public TimerClient {
public:
  int32_t init(const RobotConfig &cfg, const RobotOutInterface &interface);

  void deinit();

  void draw() const;

  FieldPos getFieldPos() const;
  Direction getDirection() const;
  void act(MoveType moveType);

  void onMoveAnimEnd(Direction futureDir, const FieldPos &futurePos);

private:
  int32_t initConfig(const RobotConfig &cfg);
  int32_t initOutInterface(const RobotOutInterface &interface);
  void onInitEnd();

  void registerCollision(const Rectangle& intersectRect,
                         CollisionDamageImpact impact) override;
  Rectangle getBoundary() const override;
  void onTimeout(const int32_t timerId) override;

  void move();

  void handleDamageImpactCollision();

  void startMoveAnim(FieldPos futurePos);
  void startRotAnim(bool isLeftRotation);
  AnimBaseConfig generateAnimBaseConfig();

  RobotConfig _state;
  RobotOutInterface _outInterface;
  Image _robotImg;

  PositionAnimation _moveAnim;
  RotationAnimation _rotAnim;
  RobotAnimEndCb _animEndCb;

  CollisionWatchStatus _currCollisionWatchStatus = CollisionWatchStatus::OFF;
};

#endif /* ROBO_COLLECTOR_GUI_ROBOT_H_ */
