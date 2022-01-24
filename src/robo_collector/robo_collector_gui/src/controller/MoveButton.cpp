//Corresponding header
#include "robo_collector_gui/controller/MoveButton.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "sdl_utils/input/InputEvent.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t MoveButton::init(const MoveButtonCfg &cfg) {
  if (nullptr == cfg.clickCb) {
    LOGERR("Error, nullptr provided for clickCb for MoveButton with rsrcId: "
        "%#16lX", cfg.rsrcId);
    return FAILURE;
  }
  _clickCb = cfg.clickCb;

  if (MoveType::UNKNOWN == cfg.moveType) {
    LOGERR("MoveType::UNKNOWN detected for MoveButton with rsrcId: %#16lX",
        cfg.rsrcId);
    return FAILURE;
  }
  _moveType = cfg.moveType;

  create(cfg.rsrcId);
  setPosition(cfg.startPos);

  const auto lightBlue = Color(0x29B6F6FF);
  _infoText.create(cfg.infoTextFontId, cfg.infoTextContent.c_str(),
      lightBlue, cfg.infoTextPos);

  return SUCCESS;
}

void MoveButton::handleEvent(const InputEvent &e) {
  if (TouchEvent::TOUCH_PRESS == e.type) {
    setFrame(CLICKED);
  } else if (TouchEvent::TOUCH_RELEASE == e.type) {
    setFrame(UNCLICKED);
    _clickCb(_moveType);
  }
}

void MoveButton::draw() const {
  ButtonBase::draw();
  _infoText.draw();
}

