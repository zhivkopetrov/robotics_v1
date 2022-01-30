#ifndef ROBO_COLLECTOR_GUI_ROBOTANIMATOR_H_
#define ROBO_COLLECTOR_GUI_ROBOTANIMATOR_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <array>

//Other libraries headers
#include "manager_utils/drawing/Image.h"
#include "manager_utils/drawing/animation/PositionAnimation.h"
#include "manager_utils/drawing/animation/RotationAnimation.h"
#include "manager_utils/time/TimerClient.h"

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiFunctionalDefines.h"
#include "robo_collector_gui/field/FieldPos.h"
#include "robo_collector_gui/entities/robot/animation/RobotAnimEndCb.h"

//Forward declarations

enum class RobotEndTurn {
  YES, NO
};

struct RobotAnimatorConfigBase {
  uint64_t rsrcId = 0;
  int32_t frameId = 0;
  int32_t moveAnimTimerId = 0;
  int32_t wallCollisionAnimTimerId = 0;
  int32_t robotCollisionAnimTimerId = 0;
};

struct RobotAnimatorConfig {
  RobotAnimatorConfigBase baseCfg;
  FieldPos startPos;
  Direction startDir;
  std::function<void(Direction, const FieldPos&)> onMoveAnimEndCb;
  std::function<void(RobotEndTurn)> collisionImpactAnimEndCb;
  std::function<void()> collisionImpactCb;
  GetRobotFieldPosCb getRobotFieldPosCb;
};

class RobotAnimator: public TimerClient {
public:
  int32_t init(const RobotAnimatorConfig &cfg);
  void draw() const;

  void startMoveAnim(const FieldPos &currPos, Direction currDir,
                     const FieldPos &futurePos);
  void stopMoveAnim();

  void startRotAnim(const FieldPos &currPos, Direction currDir,
                    RotationDir rotDir);
  void startWallCollisionTimer();

  void startCollisionImpactAnim(RobotEndTurn status);

  Rectangle getBoundary() const;

private:
  void onTimeout(const int32_t timerId) override;
  AnimBaseConfig generateAnimBaseConfig(const FieldPos &currPos);
  void processCollisionAnim();

  enum InternalDefines {
    COLLISION_ANIM_FRAMES = 8
  };

  Image _robotImg;
  PositionAnimation _moveAnim;
  RotationAnimation _rotAnim;
  RobotAnimEndCb _animEndCb;

  int32_t _moveAnimTimerId = 0;
  int32_t _wallCollisionAnimTimerId = 0;
  int32_t _robotCollisionAnimTimerId = 0;

  RobotEndTurn _collisionAnimOutcome = RobotEndTurn::YES;
  int32_t _collisionAnimStep = 0;
  int32_t _currCollisionAnimFrame = 0;
  std::array<Point, COLLISION_ANIM_FRAMES> _collisionOffsets;

  std::function<void(RobotEndTurn)> _collisionImpactAnimEndCb;
  std::function<void()> _collisionImpactCb;
  GetRobotFieldPosCb _getRobotFieldPosCb;
};

#endif /* ROBO_COLLECTOR_GUI_ROBOTANIMATOR_H_ */
