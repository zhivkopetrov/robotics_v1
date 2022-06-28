//Corresponding header
#include "robo_common/layout/debug/DebugField.h"

//System headers

//Other libraries headers
#include "utils/drawing/WidgetAligner.h"
#include "utils/Log.h"

//Own components headers

namespace {
constexpr int32_t TARGET_OPACITY = 200;
}

ErrorCode DebugField::init(const DebugFieldConfig &cfg,
                           const FieldDescription &fieldDescr,
                           const GetRobotAbsolutePosCb &getRobotAbsolutePosCb) {
  if (nullptr == getRobotAbsolutePosCb) {
    LOGERR("Error, nullptr provided for GetRobotAbsolutePosCb");
    return ErrorCode::FAILURE;
  }
  _getRobotAbsolutePosCb = getRobotAbsolutePosCb;

  const auto fieldWidth = fieldDescr.cols * fieldDescr.tileWidth;
  const auto fieldHeight = fieldDescr.rows * fieldDescr.tileHeight;
  const auto fieldDimensios = Rectangle(RoboCommonDefines::FIRST_TILE_X_POS,
      RoboCommonDefines::FIRST_TILE_Y_POS, fieldWidth, fieldHeight);

  _xDelimiter = fieldDimensios.x + (fieldDimensios.w / 2);
  _yDelimiter = fieldDimensios.y + (fieldDimensios.h / 2);

  _fbo.create(cfg.dimensions);
  _fbo.activateAlphaModulation();
  _fbo.setResetColor(Colors::FULL_TRANSPARENT);
  _fbo.setOpacity(TARGET_OPACITY);

  _bgrImg.create(cfg.panelRsrcId);
  _bgrImg.activateScaling();
  _bgrImg.setScaledWidth(cfg.dimensions.w);
  _bgrImg.setScaledHeight(cfg.dimensions.h);
  _bgrImg.activateAlphaModulation();
  _bgrImg.setOpacity(TARGET_OPACITY);

  constexpr int32_t offset = 20;
  const int32_t maxTextWidth = cfg.dimensions.w - (2 * offset);

  _msgText.create(cfg.texFotnRsrcId, " ", Colors::RED);
  _msgText.activateScaling();
  _msgText.setMaxScalingWidth(maxTextWidth);
  _msgText.activateAlphaModulation();
  _msgText.setOpacity(TARGET_OPACITY);

  setMsg("Debug text placeholder");
  updateFbo();

  return ErrorCode::SUCCESS;
}

void DebugField::draw() const {
  if (!_isActive) {
    return;
  }

  _fbo.draw();
}

void DebugField::toggleStatus() {
  _isActive = !_isActive;
}

void DebugField::setMsg(const std::string &msg) {
  _msgText.setText(msg.c_str());
  _msgText.setScaledWidth(_msgText.getFrameWidth());
  const Rectangle scaledBoundary = _msgText.getScaledRect();

  constexpr int32_t offset = 20;
  const Point textPos = WidgetAligner::getPosition(scaledBoundary.w,
      scaledBoundary.h, _fbo.getImageRect(), WidgetAlignment::CENTER_CENTER,
      Margin(offset, offset, offset, offset));
  _msgText.setPosition(textPos);

  updateFbo();
}

void DebugField::process() {
  const Point robotPos = _getRobotAbsolutePosCb();

  //TODO change position based on _xDelimiter and _yDelimiter
  _fbo.setPosition(robotPos);

  //TODO add a move animation once the transition from left to right quarter
  //     or from up to down quarter or vice-versa
  //     don't forget to use relative offset, because .setPosition is used
  //     in the process method
}

void DebugField::updateFbo() {
  const Rectangle fboScaledRect = _fbo.getScaledRect();
  _bgrImg.setPosition(fboScaledRect.x, fboScaledRect.y);

  _fbo.unlock();
  _fbo.reset();

  _fbo.addWidget(_bgrImg);
  _fbo.addWidget(_msgText);

  _fbo.update();
  _fbo.lock();
}
