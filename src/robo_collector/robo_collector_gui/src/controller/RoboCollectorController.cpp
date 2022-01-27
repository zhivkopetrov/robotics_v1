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
    const RoboCollectorControllerConfig& cfg,
    const RoboCollectorControllerOutInterface& interface) {
  _outInterface = interface;
  if (nullptr == _outInterface.robotActCb) {
    LOGERR("Error, nullptr provided for robotActCb");
    return FAILURE;
  }

  const int32_t rsrcIdsSize =
      static_cast<int32_t>(cfg.moveButtonsRsrcIds.size());
  if (cfg.maxMoveButtons != rsrcIdsSize) {
    LOGERR(
      "Error, moveButtonsRsrcIds.size() is: %d, while it should be exactly: %d",
      rsrcIdsSize, cfg.maxMoveButtons);
    return FAILURE;
  }

  MoveButtonCfg moveButtonCfg;
  moveButtonCfg.clickCb = std::bind(
    &RoboCollectorController::onMoveButtonClicked, this, std::placeholders::_1);
  moveButtonCfg.infoTextFontId = cfg.moveButtonInfoTextFontId;
  const std::array<Point, Defines::MOVE_BUTTONS_CTN> buttonsPos {
    Point(1435, 695), Point(1285, 830), Point(1585, 830)
  };
  const std::array<Point, Defines::MOVE_BUTTONS_CTN> buttonsInfoTextPos {
    Point(1470, 835), Point(1280, 965), Point(1580, 965)
  };
  const std::array<std::string, Defines::MOVE_BUTTONS_CTN> buttonsInfoTextContent {
    "Move", "Rotate Left", "Rotate Right"
  };
  const std::array<MoveType, Defines::MOVE_BUTTONS_CTN> buttonsMoveType {
    MoveType::FORWARD, MoveType::ROTATE_LEFT, MoveType::ROTATE_RIGHT
  };

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

  return SUCCESS;
}

void RoboCollectorController::draw() const {
  for (const auto &button : _moveButtons) {
    button.draw();
  }

  //TODO enable once animated
//  _horDelimiter.draw();
//  _vertDelimiter.draw();
}

void RoboCollectorController::handleEvent(const InputEvent &e) {
  for (auto& button : _moveButtons) {
    if(button.isInputUnlocked() && button.containsEvent(e)) {
      button.handleEvent(e);
      break;
    }
  }
}

void RoboCollectorController::onMoveButtonClicked(MoveType moveType) {
  for (auto& button : _moveButtons) {
    button.lockInput();
  }
  _outInterface.robotActCb(moveType);
}

void RoboCollectorController::unlockInput() {
  for (auto& button : _moveButtons) {
      button.unlockInput();
    }
}


