#ifndef ROBO_COMMON_ROBOTANIMATOR_H_
#define ROBO_COMMON_ROBOTANIMATOR_H_

//System headers
#include <cstdint>
#include <array>

//Other libraries headers
#include "manager_utils/drawing/Image.h"
#include "manager_utils/drawing/animation/PositionAnimation.h"
#include "manager_utils/drawing/animation/RotationAnimation.h"
#include "manager_utils/time/TimerClient.h"

//Own components headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"
#include "robo_common/layout/entities/robot/animation/RobotAnimEndCb.h"
#include "robo_common/layout/entities/robot/animation/PlayerDamageAnimEndCb.h"

//Forward declarations

enum class RobotEndTurn {
  YES, NO
};

enum class StartPlayerDamageAnim {
  YES, NO
};

struct RobotAnimatorConfigBase {
  uint64_t robotRsrcId = 0;
  uint64_t damageMarkerRsrcId = 0;
  int32_t frameId = 0;
  int32_t robotId = 0;
  int32_t width = 0;
  int32_t height = 0;
  int32_t moveAnimTimerId = 0;
  int32_t rotateAnimTimerId = 0;
  int32_t robotCollisionAnimTimerId = 0;
  int32_t robotDamageAnimTimerId = 0;
};

struct RobotAnimatorConfig {
  RobotAnimatorConfigBase baseCfg;
  FieldPos startPos;
  Direction startDir;
};

struct RobotAnimatorOutInterface {
  std::function<void(Direction, const FieldPos&)> onMoveAnimEndCb;
  std::function<void(RobotEndTurn)> collisionImpactAnimEndCb;
  std::function<void()> collisionImpactCb;
  GetRobotStateCb getRobotStateCb;
  GetFieldDescriptionCb getFieldDescriptionCb;
};

class RobotAnimator: public TimerClient {
public:
  ErrorCode init(const RobotAnimatorConfig &cfg,
                 const RobotAnimatorOutInterface& outInterface);
  void draw() const;

  void startMoveAnim(const FieldPos &currPos, Direction currDir,
                     const FieldPos &futurePos);
  void stopMoveAnim();

  void startRotAnim(const FieldPos &currPos, Direction currDir,
                    RotationDir rotDir);
  void stopRotAnim();

  void startCollisionImpactAnim(RobotEndTurn status);

  //stops any movement animation and rollbacks to previous fieldPos
  void cancelMove();

  Rectangle getBoundary() const;

  Point getAbsolutePos() const;

  double getRotationAngle() const;

private:
  enum class AnimationType {
    MOVE, ROTATE
  };

  ErrorCode initOutInterface(const RobotAnimatorOutInterface& outInterface);

  void onTimeout(const int32_t timerId) override;

  void configurePlayerDamageAnim();
  void onPlayerDamageAnimEnd();

  AnimBaseConfig generateAnimBaseConfig(const FieldPos &currPos,
                                        AnimationType animationType);
  void processCollisionAnim();

  //rollbacks robot visuals to its last state
  void rollbackRobotState();

  enum InternalDefines {
    COLLISION_ANIM_FRAMES = 8
  };

  Image _robotImg;
  PositionAnimation _moveAnim;
  RotationAnimation _rotAnim;
  RobotAnimEndCb _animEndCb;

  PositionAnimation _playerDamageAnim;
  PlayerDamageAnimEndCb _playerDamageAnimEndCb;
  uint64_t _damageMarkerRsrcId = 0;

  int32_t _robotId = 0;
  int32_t _moveAnimTimerId = 0;
  int32_t _rotateAnimTimerId = 0;
  int32_t _robotCollisionAnimTimerId = 0;
  int32_t _robotDamageAnimTimerId = 0;

  RobotEndTurn _collisionAnimOutcome = RobotEndTurn::YES;
  int32_t _collisionAnimStep = 0;
  int32_t _currCollisionAnimFrame = 0;
  std::array<Point, COLLISION_ANIM_FRAMES> _collisionOffsets;

  RobotAnimatorOutInterface _outInterface;
};

#endif /* ROBO_COMMON_ROBOTANIMATOR_H_ */
