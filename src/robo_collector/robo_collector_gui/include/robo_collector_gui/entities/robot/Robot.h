#ifndef ROBO_COLLECTOR_GUI_ROBOT_H_
#define ROBO_COLLECTOR_GUI_ROBOT_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "manager_utils/drawing/Image.h"
#include "manager_utils/drawing/animation/PositionAnimation.h"
#include "manager_utils/drawing/animation/RotationAnimation.h"

//Own components headers
#include "robo_collector_gui/helpers/CollisionObject.h"
#include "robo_collector_gui/entities/robot/RobotAnimEndCb.h"
#include "robo_collector_gui/defines/RoboCollectorGuiDefines.h"
#include "robo_collector_gui/defines/RoboCollectorGuiFunctionalDefines.h"
#include "robo_collector_gui/field/FieldPos.h"

//Forward declarations
class CollisionWatcher;

struct RobotCfg {
  FieldPos fieldPos;
  uint64_t rsrcId = 0;
  int32_t frameId = 0;
  int32_t animTimerId = 0;
  Direction initialDir = Direction::UP;
  char fieldMarker = '!';
  char enemyFieldMarker = '?';

  CollisionCb collisionCb;
  SetFieldDataMarkerCb setFieldDataMarkerCb;
  ResetFieldDataMarkerCb resetFieldDataMarkerCb;
  GetFieldDataCb getFieldDataCb;

  CollisionWatcher* collisionWatcher = nullptr;
};

class Robot : public CollisionObject {
public:
  int32_t init(const RobotCfg &cfg);

  void deinit();

  void draw() const;

  void act(MoveType moveType);

  FieldPos getFieldPos() const;

  // called from the animEndCb
  void setMoveData(Direction futureDir, const FieldPos &futurePos);

private:
  void registerCollision(const Rectangle& intersectRect) override;
  Rectangle getBoundary() const override;

  void move();

  void startPosAnim(FieldPos futurePos);
  void startRotAnim(bool isLeftRotation);
  AnimBaseConfig generateAnimBaseConfig();

  Image _robotImg;
  FieldPos _fieldPos;
  Direction _dir = Direction::UP;
  int32_t _animTimerId;
  char _selfFieldMarker = '!';
  char _enemyFieldMarker = '?';

  CollisionCb _collisionCb;
  SetFieldDataMarkerCb _setFieldDataMarkerCb;
  ResetFieldDataMarkerCb _resetFieldDataMarkerCb;
  GetFieldDataCb _getFieldDataCb;

  PositionAnimation _posAnim;
  RotationAnimation _rotAnim;
  RobotAnimEndCb _animEndCb;

  CollisionWatcher* _collisionWatcher = nullptr;
};

#endif /* ROBO_COLLECTOR_GUI_ROBOT_H_ */
