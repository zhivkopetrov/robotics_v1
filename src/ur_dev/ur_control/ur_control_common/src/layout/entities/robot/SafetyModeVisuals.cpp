//Corresponding header
#include "ur_control_common/layout/entities/robot/SafetyModeVisuals.h"

//System headers
#include <string>

//Other libraries headers
#include "utils/drawing/WidgetAligner.h"
#include "utils/Log.h"

//Own components headers

namespace {
std::string getRobotModeName(RobotMode robotMode) {
  switch (robotMode) {
  case RobotMode::Disconnected:
    return "Disconnected";

  case RobotMode::NoController:
    return "NoController";

  case RobotMode::ConfirmSafety:
    return "ConfirmSafety";

  case RobotMode::Booting:
    return "Booting";

  case RobotMode::PowerOff:
    return "PowerOff";

  case RobotMode::PowerOn:
    return "PowerOn";

  case RobotMode::Idle:
    return "Idle";

  case RobotMode::Backdrive:
    return "Backdrive";

  case RobotMode::Running:
    return "Running";

  case RobotMode::UpdatingFirmware:
    return "UpdatingFirmware";

  default:
    return "Unknown";
  }
}

Color getRobotModeColor(RobotMode robotMode) {
  switch (robotMode) {
  case RobotMode::Idle:
    return Colors::ORANGE;

  case RobotMode::Running:
    return Colors::GREEN;

  default:
    return Colors::BLACK;
  }
}

std::string getSafetyModeName(SafetyMode safetyMode) {
  switch (safetyMode) {
  case SafetyMode::Normal:
    return "Normal";

  case SafetyMode::Reduced:
    return "Reduced";

  case SafetyMode::ProtectiveStop:
    return "ProtectiveStop";

  case SafetyMode::Recovery:
    return "Recovery";

  case SafetyMode::SafeguardStop:
    return "SafeguardStop";

  case SafetyMode::SystemEmergencyStop:
    return "SystemEmergencyStop";

  case SafetyMode::RobotEmergencyStop:
    return "RobotEmergencyStop";

  case SafetyMode::Violation:
    return "Violation";

  case SafetyMode::Fault:
    return "Fault";

  case SafetyMode::ValidateJointId:
    return "ValidateJointId";

  case SafetyMode::UndefinedSafetyMode:
    return "UndefinedSafetyMode";

  case SafetyMode::AutomaticModeSafeguardStop:
    return "AutomaticModeSafeguardStop";

  case SafetyMode::SystemThreePositionEnablingStop:
    return "SystemThreePositionEnablingStop";

  default:
    return "Unknown";
  }
}

Color getSafetyModeColor(SafetyMode safetyMode) {
  switch (safetyMode) {
  case SafetyMode::Unknown:
    return Colors::BLACK;

  case SafetyMode::Normal:
    return Colors::GREEN;

  case SafetyMode::Reduced:
    return Colors::ORANGE;

  default:
    return Colors::RED;
  }
}
} //end anonymous namespace

ErrorCode SafetyModeVisuals::init(const Rectangle &screenBoundary,
                                  uint64_t fontRsrcId) {
  _robotModeHeaderText.create(fontRsrcId, "Robot Mode: ", Colors::BLACK);
  const Rectangle robotModeRect = _robotModeText.getImageRect();
  Point pos = WidgetAligner::getPosition(robotModeRect.w, robotModeRect.h,
      screenBoundary, WidgetAlignment::CENTER_CENTER);

  constexpr int32_t initialOffsetY = 200;
  constexpr int32_t initialOffsetX = 250;
  pos.y -= initialOffsetY;
  pos.x -= initialOffsetX;
  _robotModeHeaderText.setPosition(pos);
  _robotModeText.create(fontRsrcId, " ", Colors::BLACK);
  changeRobotMode(RobotMode::Unknown);

  pos.y += (robotModeRect.h + STATUS_VISUALS_TEXTS_Y_OFFSET);

  _safetyModeHeaderText.create(fontRsrcId, "Safety Mode: ", Colors::BLACK);
  _safetyModeHeaderText.setPosition(pos);
  _safetyModeText.create(fontRsrcId, " ", Colors::BLACK);
  changeSafetyMode(SafetyMode::Unknown);

  return ErrorCode::SUCCESS;
}

void SafetyModeVisuals::draw() const {
  _robotModeHeaderText.draw();
  _robotModeText.draw();

  _safetyModeHeaderText.draw();
  _safetyModeText.draw();
}

void SafetyModeVisuals::changeRobotMode(RobotMode mode) {
  _robotMode = mode;
  const std::string modeStr = getRobotModeName(mode);
  const Color color = getRobotModeColor(mode);
  _robotModeText.setTextAndColor(modeStr.c_str(), color);

  const Rectangle robotModeHeaderRect = _robotModeHeaderText.getImageRect();
  constexpr auto xOffset = 20;
  const Point robotModePos(
      robotModeHeaderRect.x + robotModeHeaderRect.w + xOffset,
      robotModeHeaderRect.y);
  _robotModeText.setPosition(robotModePos);
}

void SafetyModeVisuals::changeSafetyMode(SafetyMode mode) {
  _safetyMode = mode;
  const std::string modeStr = getSafetyModeName(mode);
  const Color color = getSafetyModeColor(mode);
  _safetyModeText.setTextAndColor(modeStr.c_str(), color);

  const Rectangle safetyModeHeaderRect = _safetyModeHeaderText.getImageRect();
  constexpr auto xOffset = 20;
  const Point safetyModePos(
      safetyModeHeaderRect.x + safetyModeHeaderRect.w + xOffset,
      safetyModeHeaderRect.y);
  _safetyModeText.setPosition(safetyModePos);
}

Point SafetyModeVisuals::getUpperLeftBoundaryPos() const {
  return _robotModeHeaderText.getPosition();
}