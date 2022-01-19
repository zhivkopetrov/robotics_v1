//Corresponding header
#include "robo_collector_gui/RoboCollectorGui.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "sdl_utils/input/InputEvent.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/config/RoboCollectorGuiConfig.h"

int32_t RoboCollectorGui::init(const std::any &cfg) {
  using namespace std::placeholders;

  try {
    const auto &gameCfg = std::any_cast<const RoboCollectorGuiConfig&>(cfg);
    if (SUCCESS != _field.init(gameCfg.fieldCfg)) {
      LOGERR("Error in _field.init()");
      return FAILURE;
    }

    RobotCfg robotCfg;
    robotCfg._collisionCb =
        std::bind(&Panel::decreaseHealthIndicator, &_panel, _1);
    robotCfg.rsrcId = gameCfg.robotEnemiesRsrcId;
    robotCfg.fieldPos.row = 0;
    robotCfg.fieldPos.col = 0;
    robotCfg.frameId = 0;
    for (auto &enemy : _enemies) {
      ++(robotCfg.fieldPos.col);
      if (SUCCESS != enemy.init(robotCfg)) {
        LOGERR("Error in _field.init()");
        return FAILURE;
      }
      ++robotCfg.frameId;
    }

    robotCfg.rsrcId = gameCfg.robotBlinkyRsrcId;
    robotCfg.fieldPos.row = 1;
    robotCfg.fieldPos.col = 1;
    robotCfg.frameId = 0;
    if (SUCCESS != _blinky.init(robotCfg)) {
      LOGERR("Error in _field.init()");
      return FAILURE;
    }

    if (SUCCESS != _panel.init(gameCfg.panelConfig)) {
      LOGERR("Error in _field.init()");
      return FAILURE;
    }

    constexpr auto coinOffsetFromTile = 30;
    CoinConfig coinCfg;
    coinCfg.fieldPos.row = 4;
    coinCfg.fieldPos.col = 2;
    coinCfg.tileOffset = Point(coinOffsetFromTile, coinOffsetFromTile);
    coinCfg.rsrcId = gameCfg.coinAnimRsrcId;
    coinCfg.timerId = gameCfg.cointAnimTimerId;
    if (SUCCESS != _coin.init(coinCfg)) {
      LOGERR("Error in _coin.init()");
      return FAILURE;
    }

    MoveButtonCfg moveButtonCfg;
    const std::array<Point, MOVE_BUTTONS_CTN> buttonsPos {
      Point(1435, 695), Point(1285, 860), Point(1585, 860)
    };
    const std::array<MoveType, MOVE_BUTTONS_CTN> buttonsMoveType {
      MoveType::FORWARD, MoveType::ROTATE_LEFT, MoveType::ROTATE_RIGHT
    };
    const std::array<uint64_t, MOVE_BUTTONS_CTN> buttonsRsrcIds {
      gameCfg.upMoveButtonRsrcId, gameCfg.leftMoveButtonRsrcId,
      gameCfg.rightMoveButtonRsrcId
    };
    moveButtonCfg.moveCb = std::bind(&Robot::act, &_blinky, _1);

    for (auto i = 0; i < MOVE_BUTTONS_CTN; ++i) {
      moveButtonCfg.startPos = buttonsPos[i];
      moveButtonCfg.moveType = buttonsMoveType[i];
      moveButtonCfg.rsrcId = buttonsRsrcIds[i];
      if (SUCCESS != _moveButtons[i].init(moveButtonCfg)) {
        LOGERR("Error in _moveButtons[%d].init()", i);
        return FAILURE;
      }
    }

  } catch (const std::bad_any_cast &e) {
    LOGERR("std::any_cast<GuiConfig&> failed, %s", e.what());
    return FAILURE;
  }

  return SUCCESS;
}

void RoboCollectorGui::deinit() {

}

void RoboCollectorGui::draw() const {
  _field.draw();
  _panel.draw();
  _coin.draw();
  _blinky.draw();

  for (const auto &enemy : _enemies) {
    enemy.draw();
  }
  for (const auto &button : _moveButtons) {
    button.draw();
  }
}

void RoboCollectorGui::handleEvent(const InputEvent &e) {
  for (auto& button : _moveButtons) {
    if(button.isInputUnlocked() && button.containsEvent(e)) {
      button.handleEvent(e);
      break;
    }
  }
}

