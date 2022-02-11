//Corresponding header
#include "robo_collector_gui/controller/RoboCollectorController.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "sdl_utils/input/InputEvent.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t RoboCollectorController::init(
    const RoboCollectorControllerConfig &cfg,
    const RoboCollectorControllerOutInterface &interface) {
  ;
  if (nullptr == interface.robotActCb) {
    LOGERR("Error, nullptr provided for RobotActCb");
    return FAILURE;
  }
  _robotActCb = interface.robotActCb;

  const int32_t rsrcIdsSize =
      static_cast<int32_t>(cfg.moveButtonsRsrcIds.size());
  if (cfg.maxMoveButtons != rsrcIdsSize) {
    LOGERR(
        "Error, moveButtonsRsrcIds.size() is: %d, while it should be exactly: %d",
        rsrcIdsSize, cfg.maxMoveButtons);
    return FAILURE;
  }

  MoveButtonConfig moveButtonCfg;
  moveButtonCfg.clickCb = std::bind(
      &RoboCollectorController::onMoveButtonClicked, this,
      std::placeholders::_1);
  moveButtonCfg.infoTextFontId = cfg.moveButtonInfoTextFontId;
  const std::array<Point, Defines::MOVE_BUTTONS_CTN> buttonsPos { Point(1435,
      695), Point(1285, 830), Point(1585, 830) };
  const std::array<Point, Defines::MOVE_BUTTONS_CTN> buttonsInfoTextPos { Point(
      1470, 835), Point(1280, 965), Point(1580, 965) };
  const std::array<std::string, Defines::MOVE_BUTTONS_CTN> buttonsInfoTextContent {
      "Move", "Rotate Left", "Rotate Right" };
  const std::array<MoveType, Defines::MOVE_BUTTONS_CTN> buttonsMoveType {
      MoveType::FORWARD, MoveType::ROTATE_LEFT, MoveType::ROTATE_RIGHT };

  for (auto i = 0; i < Defines::MOVE_BUTTONS_CTN; ++i) {
    moveButtonCfg.rsrcId = cfg.moveButtonsRsrcIds[i];
    moveButtonCfg.startPos = buttonsPos[i];
    moveButtonCfg.moveType = buttonsMoveType[i];
    moveButtonCfg.infoTextContent = buttonsInfoTextContent[i];
    moveButtonCfg.infoTextPos = buttonsInfoTextPos[i];
    if (SUCCESS != _moveButtons[i].init(moveButtonCfg)) {
      LOGERR("Error in _moveButtons[%d].init()", i);
      return FAILURE;
    }
  }

  _horDelimiter.create(cfg.horDelimiterRsrcId);
  _horDelimiter.setPosition(1245, 500);

  _vertDelimiter.create(cfg.vertDelimiterRsrcId);
  _vertDelimiter.setPosition(1200, 550);

  HelpButtonConfig helpBtnCfg;
  helpBtnCfg.helpActivatedCb = interface.helpActivatedCb;
  helpBtnCfg.rsrcId = cfg.helpButtonRsrcId;
  if (SUCCESS != _helpButton.init(helpBtnCfg)) {
    LOGERR("Error, _helpButton.init() failed");
    return FAILURE;
  }

  SettingsButtonConfig settingsBtnCfg;
  settingsBtnCfg.settingActivatedCb = interface.settingActivatedCb;
  settingsBtnCfg.rsrcId = cfg.settingsButtonRsrcId;
  if (SUCCESS != _settingsButton.init(settingsBtnCfg)) {
    LOGERR("Error, _helpButton.init() failed");
    return FAILURE;
  }

  return SUCCESS;
}

void RoboCollectorController::draw() const {
  for (const auto &button : _moveButtons) {
    button.draw();
  }

  _helpButton.draw();
  _settingsButton.draw();

  //TODO enable once animated
//  _horDelimiter.draw();
//  _vertDelimiter.draw();
}

void RoboCollectorController::handleEvent(const InputEvent &e) {
  for (auto &button : _moveButtons) {
    if (button.isInputUnlocked() && button.containsEvent(e)) {
      button.handleEvent(e);
      return;
    }
  }

  if (_helpButton.isInputUnlocked() && _helpButton.containsEvent(e)) {
    _helpButton.handleEvent(e);
    return;
  }

  if (_settingsButton.isInputUnlocked() && _settingsButton.containsEvent(e)) {
    _settingsButton.handleEvent(e);
    return;
  }
}

void RoboCollectorController::onMoveButtonClicked(MoveType moveType) {
  lockInput();
  _robotActCb(moveType);
}

void RoboCollectorController::lockInput() {
  for (auto &button : _moveButtons) {
    button.lockInput();
  }
}

void RoboCollectorController::unlockInput() {
  for (auto &button : _moveButtons) {
    button.unlockInput();
  }
}

