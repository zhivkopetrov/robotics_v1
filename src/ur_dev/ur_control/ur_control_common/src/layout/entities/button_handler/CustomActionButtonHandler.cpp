//Corresponding header
#include "ur_control_common/layout/entities/button_handler/CustomActionButtonHandler.h"

//System headers

//Other libraries headers
#include "urscript_common/urscript/UrScriptParser.h"
#include "utils/input/InputEvent.h"
#include "utils/log/Log.h"

//Own components headers

ErrorCode CustomActionButtonHandler::init(
  const ButtonHandlerHighLevelConfig& highLevelCfg, 
  const ButtonHandlerOutInterface& outInterface) {
  ErrorCode err = ErrorCode::SUCCESS;
  const CustomActionButtonHandlerConfig parsedCfg = [&highLevelCfg, &err]() {
    CustomActionButtonHandlerConfig localCfg;
    try {
      localCfg = 
        std::any_cast<const CustomActionButtonHandlerConfig&>(highLevelCfg.cfg);
    } catch (const std::bad_any_cast &e) {
      LOGERR(
        "std::any_cast<CustomActionButtonHandlerConfig&> failed, %s", e.what());
      err = ErrorCode::FAILURE;
    }
    return localCfg;
  }();
  if (ErrorCode::SUCCESS != err) {
    LOGERR("Error, parsing CustomActionButtonHandlerConfig failed");
    return ErrorCode::FAILURE;
  }

  const CustomActionButtonHandlerOutInterface parsedOutInterface = 
      [&outInterface, &err]() {
    CustomActionButtonHandlerOutInterface localOutInterface;
    localOutInterface.publishURScriptCb = outInterface.publishURScriptCb;
    localOutInterface.invokeDashboardServiceCb = 
      outInterface.invokeDashboardServiceCb;
    try {
      localOutInterface.customActionButtonHandlerCbs = 
        std::any_cast<const CustomActionButtonHandlerCbs&>(
          outInterface.additionalOutInterface);
    } catch (const std::bad_any_cast &e) {
      LOGERR(
        "std::any_cast<CustomActionButtonHandlerCbs&> failed, %s", e.what());
      err = ErrorCode::FAILURE;
    }
    return localOutInterface;
  }();
  if (ErrorCode::SUCCESS != err) {
    LOGERR("Error, parsing CustomActionButtonHandlerOutInterface failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initInternal(parsedCfg, parsedOutInterface)) {
    LOGERR("Error, CustomActionButtonHandler::initInternal() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

void CustomActionButtonHandler::draw() const {
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

void CustomActionButtonHandler::handleEvent(const InputEvent &e) {
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

ErrorCode CustomActionButtonHandler::setCommandButtonsLockStatus(
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

void CustomActionButtonHandler::setGripperButtonsLockStatus(
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

ErrorCode CustomActionButtonHandler::initInternal(
    const CustomActionButtonHandlerConfig &cfg,
    const CustomActionButtonHandlerOutInterface &outInterface) {
  if (ErrorCode::SUCCESS != ButtonHandler::initInternal(
        cfg.baseCfg, outInterface.invokeDashboardServiceCb)) {
    LOGERR("Error, ButtonHandler::initInternal() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initGripperButtons(
        cfg, outInterface.customActionButtonHandlerCbs.gripperButtonCbs)) {
    LOGERR("Error, initUrScriptButtons() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initCommandButtons(
        cfg, outInterface.customActionButtonHandlerCbs.commandButtonCbs)) {
    LOGERR("Error, initUrScriptButtons() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode CustomActionButtonHandler::initGripperButtons(
    const CustomActionButtonHandlerConfig &cfg,
    const CustomActionButtonCbs &buttonCbs) {
  const int32_t buttonsCbsSize = static_cast<int32_t>(buttonCbs.size());
  if (buttonsCbsSize != GRIPPER_BUTTONS_COUNT) {
    LOGERR("Error, Gripper Buttons count missmatch. Custom Callbaks passed: %d "
           "vs Expected Callbacks count: %d", buttonsCbsSize, 
           GRIPPER_BUTTONS_COUNT);
    return ErrorCode::FAILURE;
  }

  CommandButtonConfig buttonCfg;
  buttonCfg.rsrcId = cfg.baseCfg.buttonRsrcId;
  buttonCfg.fontRsrcId = cfg.baseCfg.buttonFontRsrcId;
  buttonCfg.descriptionOffsetY = 25;
  buttonCfg.descriptionColor = Color(0x29B6F6FF); //light blue

  const std::array<Point, GRIPPER_BUTTONS_COUNT> buttonPositions { 
      Point(1480, 700), Point(1330, 880), Point(1630, 880) 
  };
  const std::array<std::string, GRIPPER_BUTTONS_COUNT> buttonsDescriptions {
      "Activate gripper", "Open gripper", "Close gripper" 
  };

  for (int32_t i = 0; i < GRIPPER_BUTTONS_COUNT; ++i) {
    buttonCfg.pos = buttonPositions[i];
    buttonCfg.descriptionText = buttonsDescriptions[i];
    if (ErrorCode::SUCCESS != 
          _gripperButtons[i].init(buttonCfg, buttonCbs[i])) {
      LOGERR("Error, _gripperButtons[%d].init() failed", i);
      return ErrorCode::FAILURE;
    }
  }

  return ErrorCode::SUCCESS;
}

ErrorCode CustomActionButtonHandler::initCommandButtons(
    const CustomActionButtonHandlerConfig &cfg,
    const CustomActionButtonCbs &buttonCbs) {
  const size_t buttonsDescrSize = cfg.commandButtonsDescription.size();
  const size_t buttonsCbsSize = buttonCbs.size();
  if (buttonsCbsSize != buttonsDescrSize) {
    LOGERR("Error, Custom Actions Buttons description and callback count "
           "missmatch. Custom Callbacks passed: %zu vs Button description "
           "count: %zu", buttonsCbsSize, buttonsDescrSize);
    return ErrorCode::FAILURE;
  }

  CommandButtonConfig buttonCfg;
  buttonCfg.rsrcId = cfg.baseCfg.buttonRsrcId;
  buttonCfg.fontRsrcId = cfg.baseCfg.buttonFontRsrcId;
  buttonCfg.descriptionOffsetY = 25;
  buttonCfg.descriptionColor = Color(0x29B6F6FF); //light blue

  const auto& buttonsDescr = cfg.commandButtonsDescription;
  _commandButtons.resize(buttonsCbsSize);
  for (size_t i = 0; i < buttonsCbsSize; ++i) {
    buttonCfg.pos = buttonsDescr[i].pos;
    buttonCfg.descriptionText = buttonsDescr[i].text;
    if (ErrorCode::SUCCESS != 
          _commandButtons[i].init(buttonCfg, buttonCbs[i])) {
      LOGERR("Error, _commandButtons[%zu].init() failed", i);
      return ErrorCode::FAILURE;
    }
  }

  return ErrorCode::SUCCESS;
}