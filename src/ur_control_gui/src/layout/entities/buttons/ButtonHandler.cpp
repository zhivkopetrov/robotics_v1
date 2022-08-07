//Corresponding header
#include "ur_control_gui/layout/entities/buttons/ButtonHandler.h"

//System headers

//Other libraries headers
#include "sdl_utils/input/InputEvent.h"
#include "utils/Log.h"

//Own components headers

ErrorCode ButtonHandler::init(const ButtonHandlerConfig &cfg,
                              const ButtonHandlerOutInterface &outInterface) {
  if (ErrorCode::SUCCESS != initMotionButtons(cfg,
          outInterface.publishURScriptCb)) {
    LOGERR("Error, initMotionButtons() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initGripperButtons(cfg,
          outInterface.publishURScriptCb)) {
    LOGERR("Error, initGripperButtons() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initDashboardButtons(cfg,
          outInterface.invokeDashboardCb)) {
    LOGERR("Error, initDashboardButtons() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

void ButtonHandler::draw() const {
  for (const auto &btn : _motionButtons) {
    btn.draw();
  }

  for (const auto &btn : _gripperButtons) {
    btn.draw();
  }

  for (const auto &btn : _dashboardButtons) {
    btn.draw();
  }
}

void ButtonHandler::handleEvent(const InputEvent &e) {
  for (auto &btn : _motionButtons) {
    if (btn.isInputUnlocked() && btn.containsEvent(e)) {
      btn.handleEvent(e);
      return;
    }
  }

  for (auto &btn : _gripperButtons) {
    if (btn.isInputUnlocked() && btn.containsEvent(e)) {
      btn.handleEvent(e);
      return;
    }
  }

  for (auto &btn : _dashboardButtons) {
    if (btn.isInputUnlocked() && btn.containsEvent(e)) {
      btn.handleEvent(e);
      return;
    }
  }
}

ErrorCode ButtonHandler::initMotionButtons(
    const ButtonHandlerConfig &cfg,
    const PublishURScriptCb &publishURScriptCb) {
  const Color lightBlue = Color(0x29B6F6FF);

  UrScriptButtonConfig buttonCfg;
  buttonCfg.commandData = "test";

  CommandButtonConfig &baseCfg = buttonCfg.baseCfg;
  baseCfg.rsrcId = cfg.buttonRsrcId;
  baseCfg.fontRsrcId = cfg.buttonFontRsrcId;
  baseCfg.descriptionOffsetY = 25;
  baseCfg.descriptionColor = lightBlue;

  const std::array<Point, MOTION_BUTTONS_COUNT> buttonPositions { Point(140,
      450), Point(140, 225), Point(300, 25), Point(700, 25), Point(1100, 25),
      Point(1490, 25), Point(1650, 225), Point(1650, 450) };
  const std::array<std::string, MOTION_BUTTONS_COUNT> buttonsDescriptions {
      "Greet", "Return home (joint)", "Wake up", "Lean forward (joint)",
      "Return home (linear)", "Lean forward (linear)",
      "Pick and place (non blended)", "Pick and place (blended)", };

  for (int32_t i = 0; i < MOTION_BUTTONS_COUNT; ++i) {
    baseCfg.pos = buttonPositions[i];
    baseCfg.descriptionText = buttonsDescriptions[i];
    if (ErrorCode::SUCCESS != _motionButtons[i].init(buttonCfg,
            publishURScriptCb)) {
      LOGERR("Error, _motionButtons[%d].init() failed", i);
      return ErrorCode::FAILURE;
    }
  }

  return ErrorCode::SUCCESS;
}

ErrorCode ButtonHandler::initGripperButtons(
    const ButtonHandlerConfig &cfg,
    const PublishURScriptCb &publishURScriptCb) {
  UrScriptButtonConfig buttonCfg;
  buttonCfg.commandData = "test";

  CommandButtonConfig &baseCfg = buttonCfg.baseCfg;
  baseCfg.rsrcId = cfg.buttonRsrcId;
  baseCfg.fontRsrcId = cfg.buttonFontRsrcId;
  baseCfg.descriptionOffsetY = 25;

  const std::array<Point, GRIPPER_BUTTONS_COUNT> buttonPositions { Point(1380,
      900), Point(1680, 900) };
  const std::array<std::string, GRIPPER_BUTTONS_COUNT> buttonsDescriptions {
      "Open gripper", "Close gripper" };

  for (int32_t i = 0; i < GRIPPER_BUTTONS_COUNT; ++i) {
    baseCfg.pos = buttonPositions[i];
    baseCfg.descriptionText = buttonsDescriptions[i];
    if (ErrorCode::SUCCESS != _gripperButtons[i].init(buttonCfg,
            publishURScriptCb)) {
      LOGERR("Error, _gripperButtons[%d].init() failed", i);
      return ErrorCode::FAILURE;
    }
  }

  return ErrorCode::SUCCESS;
}

ErrorCode ButtonHandler::initDashboardButtons(
    const ButtonHandlerConfig &cfg,
    const InvokeDashboardCb &invokeDashboardCb) {
  DashboardButtonConfig buttonCfg;
  CommandButtonConfig &baseCfg = buttonCfg.baseCfg;
  baseCfg.rsrcId = cfg.buttonRsrcId;
  baseCfg.fontRsrcId = cfg.buttonFontRsrcId;
  baseCfg.descriptionOffsetY = 25;

  const std::array<Point, DASHBOARD_BUTTONS_COUNT> buttonPositions { Point(100,
      900), Point(400, 900), Point(250, 720) };
  const std::array<std::string, DASHBOARD_BUTTONS_COUNT> buttonDescriptions {
      "Power on robot", "Power off robot", "Brake release" };
  constexpr std::array<DashboardCommand, DASHBOARD_BUTTONS_COUNT> buttonCommands {
      DashboardCommand::POWER_ON_ROBOT, DashboardCommand::POWER_OFF_ROBOT,
      DashboardCommand::BRAKE_RELEASE };

  for (int32_t i = 0; i < DASHBOARD_BUTTONS_COUNT; ++i) {
    baseCfg.pos = buttonPositions[i];
    baseCfg.descriptionText = buttonDescriptions[i];
    buttonCfg.command = buttonCommands[i];
    if (ErrorCode::SUCCESS != _dashboardButtons[i].init(buttonCfg,
            invokeDashboardCb)) {
      LOGERR("Error, _dashboardButtons[%d].init() failed", i);
      return ErrorCode::FAILURE;
    }
  }

  return ErrorCode::SUCCESS;
}
