//Corresponding header
#include "ur_control_common/layout/entities/button_handler/UrScriptButtonHandler.h"

//System headers

//Other libraries headers
#include "urscript_common/urscript/UrScriptParser.h"
#include "utils/input/InputEvent.h"
#include "utils/log/Log.h"

//Own components headers

ErrorCode UrScriptButtonHandler::init(
  const ButtonHandlerHighLevelConfig& highLevelCfg, 
  const ButtonHandlerOutInterface& outInterface) {
  ErrorCode err = ErrorCode::SUCCESS;
  const UrScriptButtonHandlerConfig parsedCfg = [&highLevelCfg, &err]() {
    UrScriptButtonHandlerConfig localCfg;
    try {
      localCfg = 
        std::any_cast<const UrScriptButtonHandlerConfig&>(highLevelCfg.cfg);
    } catch (const std::bad_any_cast &e) {
      LOGERR(
        "std::any_cast<UrScriptButtonHandlerConfig&> failed, %s", e.what());
      err = ErrorCode::FAILURE;
    }
    return localCfg;
  }();
  if (ErrorCode::SUCCESS != err) {
    LOGERR("Error, parsing UrScriptButtonHandlerConfig failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initInternal(parsedCfg, outInterface)) {
    LOGERR("Error, UrScriptButtonHandler::initInternal() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

void UrScriptButtonHandler::draw() const {
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

void UrScriptButtonHandler::handleEvent(const InputEvent &e) {
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

ErrorCode UrScriptButtonHandler::setCommandButtonsLockStatus(
    const std::vector<int32_t>& lockBtnIndexes,
    const std::vector<int32_t>& unlockBtnIndexes) {
  const ErrorCode errCode = 
    sanityCheckCommandButtonsLockStatus(lockBtnIndexes, unlockBtnIndexes);
  if (ErrorCode::SUCCESS != errCode) {
    LOGERR("Error in setCommandButtonsLockStatus()");
    return ErrorCode::FAILURE;
  }

  for (const int32_t lockIdx : lockBtnIndexes) {
    _commandButtons[lockIdx].lockInput();
  }
  for (const int32_t unlockIdx : unlockBtnIndexes) {
    _commandButtons[unlockIdx].unlockInput();
  }

  return ErrorCode::SUCCESS;
}

void UrScriptButtonHandler::setGripperButtonsLockStatus(
  GripperButtonsInputStatus status) {
  if (GripperButtonsInputStatus::LOCKED == status) {
    for (auto& btn : _gripperButtons) {
      btn.lockInput();
    }
  } else {
    for (auto& btn : _gripperButtons) {
      btn.unlockInput();
    }
  }
}

ErrorCode UrScriptButtonHandler::initInternal(
  const UrScriptButtonHandlerConfig &cfg,
  const ButtonHandlerOutInterface &outInterface) {
  if (ErrorCode::SUCCESS != ButtonHandler::initInternal(
        cfg.baseCfg, outInterface.invokeDashboardServiceCb)) {
    LOGERR("Error, ButtonHandler::initInternal() failed");
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

ErrorCode UrScriptButtonHandler::initGripperButtons(
    const UrScriptButtonHandlerConfig &cfg,
    const PublishURScriptCb &publishURScriptCb) {
  std::vector<std::string> scripts;
  if (ErrorCode::SUCCESS != loadButtonScripts(
          cfg.gripperScriptFolderLocation, GRIPPER_BUTTONS_COUNT, scripts)) {
    LOGERR("Error, loadButtonScripts() failed");
    return ErrorCode::FAILURE;
  }

  UrScriptButtonConfig buttonCfg;
  CommandButtonConfig &buttonBaseCfg = buttonCfg.baseCfg;
  buttonBaseCfg.rsrcId = cfg.baseCfg.buttonRsrcId;
  buttonBaseCfg.fontRsrcId = cfg.baseCfg.buttonFontRsrcId;
  buttonBaseCfg.descriptionOffsetY = 25;
  buttonBaseCfg.descriptionColor = Color(0x29B6F6FF); //light blue

  const std::array<Point, GRIPPER_BUTTONS_COUNT> buttonPositions { 
      Point(1480, 700), Point(1330, 880), Point(1630, 880) 
  };
  const std::array<std::string, GRIPPER_BUTTONS_COUNT> buttonsDescriptions {
      "Activate gripper", "Open gripper", "Close gripper" 
  };

  for (int32_t i = 0; i < GRIPPER_BUTTONS_COUNT; ++i) {
    buttonCfg.commandData = scripts[i];
    buttonBaseCfg.pos = buttonPositions[i];
    buttonBaseCfg.descriptionText = buttonsDescriptions[i];
    if (ErrorCode::SUCCESS != _gripperButtons[i].init(
            buttonCfg, publishURScriptCb)) {
      LOGERR("Error, _gripperButtons[%d].init() failed", i);
      return ErrorCode::FAILURE;
    }
  }

  return ErrorCode::SUCCESS;
}

ErrorCode UrScriptButtonHandler::initCommandButtons(
    const UrScriptButtonHandlerConfig &cfg,
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
  baseCfg.rsrcId = cfg.baseCfg.buttonRsrcId;
  baseCfg.fontRsrcId = cfg.baseCfg.buttonFontRsrcId;
  baseCfg.descriptionOffsetY = 25;
  baseCfg.descriptionColor = Color(0x29B6F6FF); //light blue

  const auto& buttonsDescr = cfg.commandButtonsDescription;
  _commandButtons.resize(commandButtonsCount);
  for (size_t i = 0; i < commandButtonsCount; ++i) {
    buttonCfg.commandData = scripts[i];
    baseCfg.pos = buttonsDescr[i].pos;
    baseCfg.descriptionText = buttonsDescr[i].text;
    if (ErrorCode::SUCCESS != _commandButtons[i].init(
          buttonCfg, publishURScriptCb)) {
      LOGERR("Error, _commandButtons[%zu].init() failed", i);
      return ErrorCode::FAILURE;
    }
  }

  return ErrorCode::SUCCESS;
}

ErrorCode UrScriptButtonHandler::loadButtonScripts(
    const std::string &folderLocation, size_t expectedParsedScriptsCount, 
    std::vector<std::string> &outScripts) {
  if (ErrorCode::SUCCESS != UrScriptParser::parseScripts(folderLocation,
          outScripts)) {
    LOGERR("Error, UrScriptParser::parseScripts() failed");
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