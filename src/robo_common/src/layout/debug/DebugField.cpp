//Corresponding header
#include "robo_common/layout/debug/DebugField.h"

//System headers

//Other libraries headers
#include "utils/data_type/EnumClassUtils.h"
#include "utils/drawing/WidgetAligner.h"
#include "utils/Log.h"

//Own components headers

namespace {
constexpr int32_t TARGET_OPACITY = 200;

constexpr int32_t FIELD_FROM_ROBOT_OFFSET = 20;
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

  _robotWidth = fieldDescr.tileWidth;
  _robotHeight = fieldDescr.tileHeight;

  createVisuals(cfg);

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
  if (!_isActive) {
    LOGR("Debug mode is not active. Discarding received debug message: [%s]",
        msg.c_str());
    return;
  }

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
  if (!_isActive) {
    return;
  }

  const Quadrant quadrant = determineFieldQuadrant();
  changeFieldPositionFromQuadrant(quadrant);

  //TODO add a move animation once the transition from left to right quarter
  //     or from up to down quarter or vice-versa
  //     don't forget to use relative offset, because .setPosition is used
  //     in the process method
}

void DebugField::createVisuals(const DebugFieldConfig &cfg) {
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

  // bypass the check during initialization
  _isActive = true;
  setMsg("Debug text placeholder");
  _isActive = false;

  updateFbo();

  const Quadrant quadrant = determineFieldQuadrant();
  changeFieldPositionFromQuadrant(quadrant);
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

DebugField::Quadrant DebugField::determineRobotQuadrant() const {
  const Point robotPos = _getRobotAbsolutePosCb();
  const int32_t robotCenterX = robotPos.x + (_robotWidth / 2);
  const int32_t robotCenterY = robotPos.y + (_robotHeight / 2);
  if (robotCenterX <= _xDelimiter) {
    if (robotCenterY <= _yDelimiter) {
      return Quadrant::TOP_LEFT;
    }

    return Quadrant::BOTTOM_LEFT;
  }

  if (robotCenterY <= _yDelimiter) {
    return Quadrant::TOP_RIGHT;
  }

  return Quadrant::BOTTOM_RIGHT;
}

DebugField::Quadrant DebugField::determineFieldQuadrant() const {
  const Quadrant robotQuadrant = determineRobotQuadrant();
  switch (robotQuadrant) {
  case Quadrant::TOP_LEFT:
    return Quadrant::BOTTOM_RIGHT;

  case Quadrant::TOP_RIGHT:
    return Quadrant::BOTTOM_LEFT;

  case Quadrant::BOTTOM_LEFT:
    return Quadrant::TOP_RIGHT;

  case Quadrant::BOTTOM_RIGHT:
    return Quadrant::TOP_LEFT;

  default:
    LOGERR("Error, received unsupported Quadrant type: %d",
        getEnumValue(robotQuadrant));
    break;
  }

  return Quadrant::BOTTOM_RIGHT;
}

void DebugField::changeFieldPositionFromQuadrant(Quadrant quadrant) {
  const Rectangle fboBoundary = _fbo.getImageRect();
  const Point robotPos = _getRobotAbsolutePosCb();
  switch (quadrant) {
  case Quadrant::TOP_LEFT:
    _fbo.setPosition(robotPos.x - FIELD_FROM_ROBOT_OFFSET - fboBoundary.w,
        robotPos.y - FIELD_FROM_ROBOT_OFFSET - fboBoundary.h);
    break;
  case Quadrant::TOP_RIGHT:
    _fbo.setPosition(robotPos.x + FIELD_FROM_ROBOT_OFFSET + _robotWidth,
        robotPos.y - FIELD_FROM_ROBOT_OFFSET - fboBoundary.h);
    break;
  case Quadrant::BOTTOM_LEFT:
    _fbo.setPosition(robotPos.x - FIELD_FROM_ROBOT_OFFSET - fboBoundary.w,
        robotPos.y + FIELD_FROM_ROBOT_OFFSET + _robotHeight);
    break;
  case Quadrant::BOTTOM_RIGHT:
    _fbo.setPosition(robotPos.x + FIELD_FROM_ROBOT_OFFSET + _robotWidth,
        robotPos.y + FIELD_FROM_ROBOT_OFFSET + _robotHeight);
    break;

  default:
    LOGERR("Error, received unsupported Quadrant type: %d",
        getEnumValue(quadrant));
    break;
  }
}
