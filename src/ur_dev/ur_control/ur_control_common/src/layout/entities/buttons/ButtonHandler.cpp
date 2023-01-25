//Corresponding header
#include "ur_control_common/layout/entities/buttons/ButtonHandler.h"

//System headers

//Other libraries headers
#include "sdl_utils/input/InputEvent.h"
#include "utils/Log.h"

//Own components headers
#include "ur_control_common/helpers/ScriptParser.h"

ErrorCode ButtonHandler::init(const ButtonHandlerConfig &cfg,
                              const ButtonHandlerOutInterface &outInterface) {
  if (ErrorCode::SUCCESS != initDashboardButtons(cfg,
          outInterface.invokeDashboardCb)) {
    LOGERR("Error, initDashboardButtons() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != 
          initGripperButtons(cfg, outInterface.publishURScriptCb)) {
    LOGERR("Error, initUrScriptButtons() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != 
          initCommandButtons(cfg, outInterface.publishURScriptCb)) {
    LOGERR("Error, initUrScriptButtons() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

void ButtonHandler::draw() const {
  for (const auto &btn : _dashboardButtons) {
    btn.draw();
  }

  for (const auto &btn : _gripperButtons) {
    btn.draw();
  }

  for (const auto &btn : _commandButtons) {
    btn.draw();
  }
}

void ButtonHandler::handleEvent(const InputEvent &e) {
  for (auto &btn : _dashboardButtons) {
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

  for (auto &btn : _commandButtons) {
    if (btn.isInputUnlocked() && btn.containsEvent(e)) {
      btn.handleEvent(e);
      return;
    }
  }
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

ErrorCode ButtonHandler::initGripperButtons(
    const ButtonHandlerConfig &cfg,
    const PublishURScriptCb &publishURScriptCb) {
  std::vector<std::string> scripts;
  if (ErrorCode::SUCCESS != loadButtonScripts(
          cfg.gripperScriptFolderLocation, GRIPPER_BUTTONS_COUNT, scripts)) {
    LOGERR("Error, loadButtonScripts() failed");
    return ErrorCode::FAILURE;
  }

  UrScriptButtonConfig buttonCfg;
  CommandButtonConfig &baseCfg = buttonCfg.baseCfg;
  baseCfg.rsrcId = cfg.buttonRsrcId;
  baseCfg.fontRsrcId = cfg.buttonFontRsrcId;
  baseCfg.descriptionOffsetY = 25;
  baseCfg.descriptionColor = Color(0x29B6F6FF); //light blue

  const std::array<Point, GRIPPER_BUTTONS_COUNT> buttonPositions { 
      Point(1480, 700), Point(1330, 880), Point(1630, 880) 
  };
  const std::array<std::string, GRIPPER_BUTTONS_COUNT> buttonsDescriptions {
      "Activate gripper", "Open gripper", "Close gripper" 
  };

  for (int32_t i = 0; i < GRIPPER_BUTTONS_COUNT; ++i) {
    buttonCfg.commandData = scripts[i];
    baseCfg.pos = buttonPositions[i];
    baseCfg.descriptionText = buttonsDescriptions[i];
    if (ErrorCode::SUCCESS != _gripperButtons[i].init(
            buttonCfg, publishURScriptCb)) {
      LOGERR("Error, _gripperButtons[%d].init() failed", i);
      return ErrorCode::FAILURE;
    }
  }

  return ErrorCode::SUCCESS;
}

ErrorCode ButtonHandler::initCommandButtons(
    const ButtonHandlerConfig &cfg,
    const PublishURScriptCb &publishURScriptCb) {
  const size_t commandButtonsCount = cfg.commandButtonsDescription.size();
  std::vector<std::string> scripts;
  if (ErrorCode::SUCCESS != loadButtonScripts(cfg.commandScriptsFolderLocation, 
          commandButtonsCount, scripts)) {
    LOGERR("Error, loadButtonScripts() failed");
    return ErrorCode::FAILURE;
  }

  UrScriptButtonConfig buttonCfg;
  CommandButtonConfig &baseCfg = buttonCfg.baseCfg;
  baseCfg.rsrcId = cfg.buttonRsrcId;
  baseCfg.fontRsrcId = cfg.buttonFontRsrcId;
  baseCfg.descriptionOffsetY = 25;
  baseCfg.descriptionColor = Color(0x29B6F6FF); //light blue

  const auto& buttonsDescr = cfg.commandButtonsDescription;
  _commandButtons.resize(commandButtonsCount);
  for (size_t i = 0; i < commandButtonsCount; ++i) {
    buttonCfg.commandData = scripts[i];
    baseCfg.pos = buttonsDescr[i].pos;
    baseCfg.descriptionText = buttonsDescr[i].text;
    if (ErrorCode::SUCCESS != _commandButtons[i].init(buttonCfg,
            publishURScriptCb)) {
      LOGERR("Error, _commandButtons[%zu].init() failed", i);
      return ErrorCode::FAILURE;
    }
  }

  return ErrorCode::SUCCESS;
}

ErrorCode ButtonHandler::loadButtonScripts(
    const std::string &folderLocation, size_t expectedParsedScriptsCount, 
    std::vector<std::string> &outScripts) {
  if (ErrorCode::SUCCESS != ScriptParser::parseScripts(folderLocation,
          outScripts)) {
    LOGERR("Error, ScriptParser::parseScripts() failed");
    return ErrorCode::FAILURE;
  }

  const size_t parsedScriptsCount = outScripts.size();
  if (expectedParsedScriptsCount != parsedScriptsCount) {
    LOGERR("Error, Scripts count missmatch. Scripts parsed: %zu vs "
           "Expected scripts count: %zu", parsedScriptsCount, 
           expectedParsedScriptsCount);
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}