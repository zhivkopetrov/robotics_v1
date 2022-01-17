//Corresponding header
#include "robo_collector_gui/entities/Robot.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "sdl_utils/input/InputEvent.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t Robot::init(const RobotCfg& cfg) {
  _img.create(cfg.rsrcId);
  _img.setPosition(cfg.startPos);
  _img.setFrame(cfg.frameId);
  return SUCCESS;
}

void Robot::draw() const {
  _img.draw();
}

void Robot::handleEvent(const InputEvent& e) {
  if (TouchEvent::KEYBOARD_RELEASE != e.type) {
    return;
  }

  switch (e.key) {
  case Keyboard::KEY_RIGHT:
    _img.moveRight(_img.getFrameWidth());
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
