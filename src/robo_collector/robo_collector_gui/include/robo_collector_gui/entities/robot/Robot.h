#ifndef ROBO_COLLECTOR_GUI_ROBOT_H_
#define ROBO_COLLECTOR_GUI_ROBOT_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <functional>

//Other libraries headers
#include "manager_utils/drawing/Image.h"
#include "manager_utils/drawing/animation/PositionAnimation.h"
#include "manager_utils/drawing/animation/RotationAnimation.h"

//Own components headers
#include "robo_collector_gui/entities/robot/RobotAnimEndCb.h"
#include "robo_collector_gui/defines/RoboCollectorGuiDefines.h"
#include "robo_collector_gui/field/FieldPos.h"

//Forward declarations

struct RobotCfg {
  FieldPos fieldPos;
  uint64_t rsrcId = 0;
  int32_t frameId = 0;
  int32_t animTimerId = 0;
  std::function<void(int32_t)> _collisionCb;
};

class Robot {
public:
  int32_t init(const RobotCfg& cfg);

  void draw() const;

  void act(MoveType moveType);

  FieldPos getFieldPos() const;

  // called from the animEndCb
  void setMoveData(Direction futureDir, const FieldPos& futurePos);

private:
  void move();

  void startPosAnim(FieldPos futurePos);
  void startRotAnim(bool isLeftRotation);
  AnimBaseConfig generateAnimBaseConfig();

  Image _robotImg;
  FieldPos _fieldPos;
  Direction _dir = Direction::UP;
  std::function<void(int32_t)> _collisionCb;
  int32_t _animTimerId;

  PositionAnimation _posAnim;
  RotationAnimation _rotAnim;
  RobotAnimEndCb _animEndCb;
};

#endif /* ROBO_COLLECTOR_GUI_ROBOT_H_ */
