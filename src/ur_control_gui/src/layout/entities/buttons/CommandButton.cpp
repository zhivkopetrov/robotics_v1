//Corresponding header
#include "ur_control_gui/layout/entities/buttons/CommandButton.h"

//System headers

//Other libraries headers
#include "sdl_utils/input/InputEvent.h"
#include "utils/drawing/WidgetAligner.h"
#include "utils/Log.h"

//Own components headers

ErrorCode CommandButton::init(const CommandButtonConfig &cfg) {
  ButtonBase::create(cfg.rsrcId);
  ButtonBase::setPosition(cfg.pos);

  _description.create(cfg.fontRsrcId, cfg.descriptionText.c_str(),
      cfg.descriptionColor);
  const Rectangle buttonDimensions = ButtonBase::getButtonRect();
  const Point centeredPos = WidgetAligner::getPosition(
      _description.getFrameWidth(), _description.getFrameHeight(),
      buttonDimensions, WidgetAlignment::LOWER_CENTER);

  _description.setPosition(centeredPos);
  _description.moveDown(cfg.descriptionOffsetY);

  return ErrorCode::SUCCESS;
}

void CommandButton::draw() const {
  ButtonBase::draw();
  _description.draw();
}

