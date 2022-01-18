//Corresponding header
#include "robo_collector_gui/entities/Robot.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "sdl_utils/input/InputEvent.h"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/field/FieldUtils.h"

int32_t Robot::init(const RobotCfg& cfg) {
  _img.create(cfg.rsrcId);
  _img.setPosition(FieldUtils::getAbsPos(cfg.fieldPos));
  _img.setFrame(cfg.frameId);
  _fieldPos = cfg.fieldPos;

  return SUCCESS;
}

void Robot::draw() const {
  _img.draw();
}

void Robot::handleEvent(const InputEvent& e) {
  if (TouchEvent::KEYBOARD_RELEASE != e.type) {
    return;
  }

  //TODO remove me
  switch (e.key) {
  case Keyboard::KEY_RIGHT:
    _img.moveRight(_img.getFrameHeight());
    break;
  case Keyboard::KEY_DOWN:
    _img.moveDown(_img.getFrameHeight());
    break;
  case Keyboard::KEY_LEFT:
    _img.moveLeft(_img.getFrameWidth());
    break;
  case Keyboard::KEY_UP:
    _img.moveUp(_img.getFrameHeight());
    break;

  default:
    break;
  }
}

void Robot::move(MoveType moveType) {
  switch (moveType) {
  case MoveType::FORWARD:
    _fieldPos = FieldUtils::getAdjacentPos(_dir, _fieldPos);
    _img.setPosition(FieldUtils::getAbsPos(_fieldPos));
    break;

    //TODO rotate and not move
  case MoveType::ROTATE_LEFT:
    _img.moveLeft(_img.getFrameWidth());
    break;
  case MoveType::ROTATE_RIGHT:
    _img.moveRight(_img.getFrameWidth());
    break;

  default:
    LOGERR("Error, received unsupported moveType: %d", getEnumValue(moveType));
    break;
  }
}

FieldPos Robot::getFieldPos() const {
  return _fieldPos;
}


