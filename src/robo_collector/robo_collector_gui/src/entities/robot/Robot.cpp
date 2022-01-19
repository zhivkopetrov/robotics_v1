//Corresponding header
#include "robo_collector_gui/entities/robot/Robot.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/data_type/EnumClassUtils.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/field/FieldUtils.h"
#include "robo_collector_gui/entities/robot/RobotUtils.h"

int32_t Robot::init(const RobotCfg& cfg) {
  if (nullptr == cfg._collisionCb) {
    LOGERR("Error, nullptr provided for RobotCfg collisionCb");
    return FAILURE;
  }
  _collisionCb = cfg._collisionCb;

  _img.create(cfg.rsrcId);
  _img.setPosition(FieldUtils::getAbsPos(cfg.fieldPos));
  _img.setFrame(cfg.frameId);
  _img.setPredefinedRotationCenter(RotationCenterType::ORIG_CENTER);

  _fieldPos = cfg.fieldPos;

  return SUCCESS;
}

void Robot::draw() const {
  _img.draw();
}

void Robot::act(MoveType moveType) {
  switch (moveType) {
  case MoveType::FORWARD:
    move();
    break;

  case MoveType::ROTATE_LEFT:
    rotate(true /*isLeftRotation*/);
    break;
  case MoveType::ROTATE_RIGHT:
    rotate(false /*isLeftRotation*/);
    break;

  default:
    LOGERR("Error, received unsupported moveType: %d", getEnumValue(moveType));
    break;
  }
}

FieldPos Robot::getFieldPos() const {
  return _fieldPos;
}

void Robot::move() {
  const auto futurePos = FieldUtils::getAdjacentPos(_dir, _fieldPos);
  if (FieldUtils::isInsideField(futurePos)) {
    _fieldPos = futurePos;
  } else {
    constexpr auto damage = 10;
    _collisionCb(damage);
  }

  _img.setPosition(FieldUtils::getAbsPos(_fieldPos));
}

void Robot::rotate(bool isLeftRotation) {
  _dir = RobotUtils::getDirAfterRotation(_dir, isLeftRotation);
  _img.setRotation(RobotUtils::getRotationDegFromDir(_dir));
}


