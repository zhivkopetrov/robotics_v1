//Corresponding header
#include "ur_control_gui/layout/entities/buttons/ButtonHandler.h"

//System headers

//Other libraries headers
#include "sdl_utils/input/InputEvent.h"
#include "utils/Log.h"

//Own components headers
#include "ur_control_gui/helpers/ScriptParser.h"

ErrorCode ButtonHandler::init(const ButtonHandlerConfig &cfg,
                              const ButtonHandlerOutInterface &outInterface) {

  std::vector<std::string> scripts;
  if (ErrorCode::SUCCESS != loadButtonScripts(cfg.scriptFolderLocation,
          scripts)) {
    LOGERR("Error, loadButtonScripts() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initUrScriptButtons(cfg,
          outInterface.publishURScriptCb)) {
    LOGERR("Error, initUrScriptButtons() failed");
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
  for (const auto &btn : _urscriptButtons) {
    btn.draw();
  }

  for (const auto &btn : _dashboardButtons) {
    btn.draw();
  }
}

void ButtonHandler::handleEvent(const InputEvent &e) {
  for (auto &btn : _urscriptButtons) {
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

ErrorCode ButtonHandler::loadButtonScripts(
    const std::string &folderLocation, std::vector<std::string> &outScripts) {
  if (ErrorCode::SUCCESS != ScriptParser::parseScripts(folderLocation,
          outScripts)) {
    LOGERR("Error, ScriptParser::parseScripts() failed");
    return ErrorCode::FAILURE;
  }

  const int32_t count = static_cast<int32_t>(outScripts.size());
  if (count != URSCRIPT_BUTTONS_COUNT) {
    LOGERR("Error, Scripts count missmatch. Scripts parsed: %d vs "
           "ScriptButtons: %d", count, URSCRIPT_BUTTONS_COUNT);
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode ButtonHandler::initUrScriptButtons(
    const ButtonHandlerConfig &cfg,
    const PublishURScriptCb &publishURScriptCb) {
  const Color lightBlue = Color(0x29B6F6FF);

  UrScriptButtonConfig buttonCfg;
  buttonCfg.commandData =
      "movel(p[-0.49,-0.575,0.576,2.16,2.19,0],a=0.1,v=0.1,t=0,r=0)";

  CommandButtonConfig &baseCfg = buttonCfg.baseCfg;
  baseCfg.rsrcId = cfg.buttonRsrcId;
  baseCfg.fontRsrcId = cfg.buttonFontRsrcId;
  baseCfg.descriptionOffsetY = 25;
  baseCfg.descriptionColor = lightBlue;

  const std::array<Point, URSCRIPT_BUTTONS_COUNT> buttonPositions { Point(100,
      450), Point(100, 225), Point(300, 25), Point(650, 25), Point(1000, 25),
      Point(1370, 25), Point(1545, 225), Point(1545, 450), Point(1480, 700),
      Point(1330, 880), Point(1630, 880) };
  const std::array<std::string, URSCRIPT_BUTTONS_COUNT> buttonsDescriptions {
      "Greet", "Return home (joint)", "Wake up", "Lean forward (joint)",
      "Return home (linear)", "Lean forward (linear)",
      "Pick and place (non blended)", "Pick and place (blended)",
      "Activate gripper", "Open gripper", "Close gripper" };

  for (int32_t i = 0; i < URSCRIPT_BUTTONS_COUNT; ++i) {
    baseCfg.pos = buttonPositions[i];
    baseCfg.descriptionText = buttonsDescriptions[i];
    if (ErrorCode::SUCCESS != _urscriptButtons[i].init(buttonCfg,
            publishURScriptCb)) {
      LOGERR("Error, _urscriptButtons[%d].init() failed", i);
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
      880), Point(400, 880), Point(250, 700) };
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
