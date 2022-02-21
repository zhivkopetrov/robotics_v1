//Corresponding header
#include "robo_collector_common/layout/controller/RoboCollectorUiController.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "sdl_utils/input/InputEvent.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t RoboCollectorUiController::init(
    const RoboCollectorUiControllerConfig &cfg,
    const RoboCollectorUiControllerOutInterface &interface) {
  if (nullptr == interface.robotActCb) {
    LOGERR("Error, nullptr provided for RobotActCb");
    return FAILURE;
  }
  _robotActCb = interface.robotActCb;

  const auto clickCb = std::bind(
      &RoboCollectorUiController::onMoveButtonClicked, this,
      std::placeholders::_1);

  const auto moveButtonsSize = cfg.moveButtonsCfgs.size();
  _moveButtons.resize(moveButtonsSize);
  for (size_t i = 0; i < moveButtonsSize; ++i) {
    if (SUCCESS != _moveButtons[i].init(cfg.moveButtonsCfgs[i], clickCb)) {
      LOGERR("Error in _moveButtons[%zu].init()", i);
      return FAILURE;
    }
  }

  //TODO extract hardcoded coordinates
  _horDelimiter.create(cfg.horDelimiterRsrcId);
  _horDelimiter.setPosition(1245, 500);
  _vertDelimiter.create(cfg.vertDelimiterRsrcId);
  _vertDelimiter.setPosition(1200, 550);

  if (SUCCESS != _helpButton.init(cfg.helpBtnCfg, interface.helpActivatedCb)) {
    LOGERR("Error, _helpButton.init() failed");
    return FAILURE;
  }

  if (SUCCESS !=
      _settingsButton.init(cfg.settingsBtnCfg, interface.settingActivatedCb)) {
    LOGERR("Error, _helpButton.init() failed");
    return FAILURE;
  }

  _isEnabled = cfg.isEnabled;

  return SUCCESS;
}

void RoboCollectorUiController::draw() const {
  for (const auto &button : _moveButtons) {
    button.draw();
  }

  _helpButton.draw();
  _settingsButton.draw();

  //TODO enable once animated
//  _horDelimiter.draw();
//  _vertDelimiter.draw();
}

void RoboCollectorUiController::handleEvent(const InputEvent &e) {
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

void RoboCollectorUiController::onMoveButtonClicked(MoveType moveType) {
  lockInput();
  _robotActCb(moveType);
}

void RoboCollectorUiController::lockInput() {
  for (auto &button : _moveButtons) {
    button.lockInput();
  }
}

void RoboCollectorUiController::unlockInput() {
  for (auto &button : _moveButtons) {
    button.unlockInput();
  }
}

bool RoboCollectorUiController::isEnabled() const {
  return _isEnabled;
}

